clear all

% FSS + beam system parameters
m1 = 10;    % kg
m2 = 2;   % kg
l1 = 0.27;  % m FSS side
l2 = 0.4;   % m beam length
l3 = 0.005; % m beam thickness
k = 0.5;     % N/rad spring constant

M = m1+m2;
i1 = 1/6*m1*l1^2;
i2 = 1/12*m2*(l2^2+l3^2);

% Initial conditions
r0 = 0;             % m
s0 = 0;             % m
theta10 = 0;       % rad
theta20 = 0.9;       % rad
rdot0 = 0.1;         % m/s
sdot0 = 0;         % m/s
theta1dot0 = 0;    % rad/s
theta2dot0 = 0;    % rad/sec
% Problem parameters
t0 = 0; % start time
tf = 15; % final time
dt = .1; % time increment between control updates

% Start MPC loop
% Initialize loop parameters
tk = t0;
theta1_true = [];
theta2_true = [];
T_true = [];
Torque_history = [];
D = diag([0 0 0 -k]);
yk = [r0 s0 theta10 theta20 rdot0 sdot0 theta1dot0 theta2dot0];
yksmall = [theta10 theta20 theta1dot0 theta2dot0];
options = odeset('reltol',1e-12,'abstol',1e-12);

% Define the NLMPC object
nx = 8; % Number of States: 
nu = 4; % Number of Inputs
ny = 8; % Number of Outputs

% NLMPC controller  
nlobj = nlmpc(nx,ny,nu);

% Define Sample Time
nlobj.Ts = dt; 

% Define Prediction and Control Horizons
nlobj.PredictionHorizon = 10; 
nlobj.ControlHorizon = 2; 

% Define state and output functions
nlobj.Model.StateFcn = @(x, u) MyStateFcn(x, u, m1, m2, l1, l2, i1, i2, k,M);
nlobj.Model.OutputFcn = @(x, u) MyOutputFcn(x, u); 

% Initialize Controller States
x0 = [r0; s0; theta10; theta20; rdot0; sdot0; theta1dot0; theta2dot0]; % Initialize all states
u0 = [0; 0; 0; 0];  
validateFcns(nlobj, x0, u0); 

% Loop
while tk<tf
    % Calculate H and C at tk
    theta2k = yk(4);
    theta1dotk = yk(7);
    theta2dotk = yk(8);
    H = findH(m1,m2,l1,l2,i1,i2,theta2k);
    C = findC(m1,m2,l1,l2,theta2k,theta1dotk,theta2dotk);


    % compute control input using NLMPC
    measurements = yksmall; 
    lastMV = [0;0;0;0]; % initialize lastMV if it is not initialized
    [mv, info] = nlmpcmove(nlobj, x0, measurements);
    Torque = mv; % The computed control input
    
   % Simulate the system
    tspan = [tk tk+dt]; % s
    [T, Y] = ode45(@(t,y) MyStateFcn(y, Torque, m1, m2, l1, l2, i1, i2, k, M), tspan, yk, options);
    
    % Store the results
    theta1_true = [theta1_true; Y(:,3)];
    theta2_true = [theta2_true; Y(:,4)];
    T_true = [T_true; T];
    Torque_history = [Torque_history; repmat(Torque', length(T), 1)];
    
    % Update the initial state 
    yk = Y(end, :); 
    x0 = yk'; % Update x0 to the new state
   
    tk = T(end);
end

figure;
subplot(2,1,1);
plot(T_true, theta1_true);
title('Theta1 over Time');
xlabel('Time (s)');
ylabel('Theta1 (rad)');

subplot(2,1,2);
plot(T_true, theta2_true);
title('Theta2 over Time');
xlabel('Time (s)');
ylabel('Theta2 (rad)');

figure;
plot(T_true, Torque_history);
title('Torque over Time');
xlabel('Time (s)');
ylabel('Torque (N·m)');


function dxdt = MyStateFcn(x, u, m1, m2, l1, l2, i1, i2, k, M)
    
    % Extracting states.
    theta2 = x(4);
    theta1dot = x(7);
    theta2dot = x(8);
    M = m1+m2;
    
    % Finding C and H matrices.
    H33 = (1/(4*M)*(m1+m2)*(l1^2+l2^2+2*l1*l2*cos(theta2))) + (i1+i2);
    H34 = (1/(4*M)*(m1+m2)*(l2^2+l1*l2*cos(theta2))) + i2; % Corrected H34
    H44 = (1/(4*M))*((m1+m2)*l2^2) + i2;
    H = [M 0 0 0; 0 M 0 0; 0 0 H33 H34; 0 0 H34 H44];
    
    C33 = -(1/(4*M))*(m1*m2*l1*l2*sin(theta2)*theta2dot);
    C43 = (1/(4*M))*(m1*m2*l1*l2*sin(theta2)*theta1dot);
    C = [0 0 0 0; 0 0 0 0; 0 0 C33 (C33-C43); 0 0 C43 0];
    
    v = x(5:8, 1); 
    G = [0; 0; 0; -k * x(4)]; 
    %F = [u(1); u(2); u(3); u(4)]; 
    F = [0;0;u(1);0];
    
    % Solving for q_ddot 
    q_ddot = H \ (G + F - C*v); 
    
    dxdt = [x(5:8); q_ddot]; 
end

function y = MyOutputFcn(x, u)
    y = x; 
end




