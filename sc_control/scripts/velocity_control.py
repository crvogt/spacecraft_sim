import numpy as np
import rospy
import tf.transformations as trans

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped 
from std_msgs.msg import Float32
from PID import PIDControllerBase 

from gazebo_msgs.msg import ModelStates

class PositionControllerNode:
    def __init__(self, namespace="polysat"):
        print("PositionControllerNode: initializing node")

        # Set information variables
        self.namespace = namespace
        self.getStateMsg = ModelStates()
        self.getImuMsg = Imu()
        self.getPosCmd = PoseStamped() 
        self.poseVal = PoseStamped()
        self.t_val = 0
        self.num_thrusters = 14
        self.msg_vals = np.asarray([0.0] * self.num_thrusters)
        
        # Set PID values
        # Simple split
        # R, P, Y, x, y, z
        # y becomes unstable at p=10 so we set it to 10/2
        self.p_vals = np.array([0, 0, 0, 0, 5500.0, 0])
        self.i_vals = np.array([0, 0, 0, 0, 11000.0, 0])
        self.d_vals = np.array([0, 0, 0, 0, 10.0, 0])
        self.sat_vals = np.array([1, 1, 1, 1, 250, 1])
        self.pid_list = [0] * 6 
        self.set_pid_vals()
        self.pid_out = [0] * 6

        # Putting gain dicts here...
        self.gain_dict = dict()
        self.gain_dict[0] = np.asarray([0, 0, 0, 0, -1.0, 1.0, 1.0, -1.0, 0, 0, 0, 0, 0, 0])
        self.gain_dict[1] = np.asarray([1.0, -1.0, -1.0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.gain_dict[2] = np.asarray([0, 0, 0, 0, 0, 0, 0, 0, -1.0, 1.0, 1.0, -1.0, 0, 0])
        self.gain_dict[3] = np.asarray([1.0, 1.0, -1.0, -1.0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        self.gain_dict[4] = np.asarray([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1.0, -1.0])
        self.gain_dict[5] = np.asarray([0, 0, 0, 0, -0.5, -0.5, -0.5, -0.5, 0, 0, 0, 0, 0, 0])
        
        # Set flags
        self.initialized = False
        # Set this to false till we can determine 3D pose
        self.use_imu = False

        # ROS
        self.thrust_msg = Float32()
        self.sub_cmd_pose = rospy.Subscriber('cmd_pose', PoseStamped, self.cmd_pose_callback)
        self.position_sub = rospy.Subscriber('polysat/imu', Imu, self.odometry_callback)
        self.model_state_sub = rospy.Subscriber('gazebo/model_states', ModelStates, self.state_callback)
        # Add thruster subs
        self.thruster_pubs()

    def cmd_pose_callback(self, _msg):
        # Store the desired pose
        self.getPosCmd = _msg
        self.t_val = _msg.header.stamp.to_sec()
        #print(self.t_val)

    def odometry_callback(self, _msg):
        self.getImuMsg = _msg
        self.t_val = _msg.header.stamp.to_sec()
        if self.use_imu:
            self.control_pose()

    def state_callback(self, _msg):
        self.getStateMsg = _msg
        if not self.use_imu:
            self.poseVal.pose = self.getStateMsg.pose[-1]
            self.control_pose()

    def control_pose(self):
        # Angular error
        # could get the quat diff, then get the angle from the quaternion...
        cmd_angles = trans.euler_from_quaternion([self.getPosCmd.pose.orientation.x,
                                                  self.getPosCmd.pose.orientation.y,
                                                  self.getPosCmd.pose.orientation.z, 
                                                  self.getPosCmd.pose.orientation.w])
        cur_angles = trans.euler_from_quaternion([self.poseVal.pose.orientation.x,
                                                  self.poseVal.pose.orientation.y,
                                                  self.poseVal.pose.orientation.z,
                                                  self.poseVal.pose.orientation.w])
        Rol_err = cmd_angles[0] - cur_angles[0] 
        Pit_err = cmd_angles[1] - cur_angles[1] 
        Yaw_err = cmd_angles[2] - cur_angles[2] 

        # Spatial error
        cmd_pose = self.getPosCmd.pose.position
        cur_pose = self.poseVal.pose.position
        x_err = cmd_pose.x - cur_pose.x
        y_err = cmd_pose.y - cur_pose.y  
        z_err = cmd_pose.z - cur_pose.z  

        error_list = [Rol_err, Pit_err, Yaw_err,
                      x_err, y_err, z_err]

        # Send values to regulator
        for ii, pid_reg in enumerate(self.pid_list):
            self.t_val = rospy.Time.now().to_sec()
            self.pid_out[ii] = pid_reg.regulate(error_list[ii], self.t_val)

        self.publish_vals()

    def publish_vals(self):
        # multiply gains 
        for ii in range(len(self.msg_vals)):
            self.msg_vals[ii] = 0.0
        for ii, pid_val in enumerate(self.pid_out):
            self.msg_vals += self.gain_dict[ii] * pid_val

        for ii, thruster in enumerate(self.thrust_list):
            self.thrust_msg.data = self.msg_vals[ii]
            thruster.publish(self.thrust_msg)

    def thruster_pubs(self):
        self.thrust_list = [rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 0), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 1), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 2), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 3), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 4), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 5), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 6), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 7), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 8), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 9), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 10), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 11), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 12), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(self.namespace, 13), Float32, queue_size=1)]

    def set_pid_vals(self):
        print("Initializing PID values per DoF")
        for ii in range(6):
            self.pid_list[ii] = PIDControllerBase(self.p_vals[ii],
                                             self.i_vals[ii],
                                             self.d_vals[ii],
                                             self.sat_vals[ii])

if __name__=="__main__":
    print('Starting PositionControl.py')
    rospy.init_node('position_control')

    try:
        node = PositionControllerNode(namespace='polysat')
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Caught an exception')
    print('exciting')
        
