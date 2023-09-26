import matplotlib.pyplot as plt
import numpy as np

'''
For TimeTorque_p1.txt
15.0198
15.8076
16.598
if t <= 15.0198, torque = -0.005
if t > 15.0198 and t <= 15.8076, torque = 0.005
if t > 15.8076 and t <= 16.598, torque = -0.005
if t > 16.598 and t <= 31.6153, torque = 0.005
if t > 31.6153, torque = 0
'''

'''
For TimeTorque_p2.txt

if t <= 17.259037, torque = -0.005
if t > 17.259037 and t <= 20.11859, torque = 0.005
if t > 20.11859 and t <= 21.1105, torque = -0.005
if t > 21.1105 and t <= 34.43167, torque = 0.005
if t > 34.431672, torque = 0
'''

def findDeriv(time, values):
    prev_val = 0
    cur_val = 0
    prev_time = 0
    cur_time = 0
    for ii, val in enumerate(values):
        if ii == 0:
            prev_val = val
            prev_time = time[ii]
        else:
            deriv = (val - prev_val)/(time[ii] - prev_time)
            if abs(deriv) > 0:
                print("deriv {}, time {}, val {}".format(deriv, prev_time, prev_val))
            prev_time = time[ii]
            prev_val = val
            if abs(deriv) > 1:
                pass
                # print("time {}, val {}".format(prev_time, prev_val))


filename = "TimeTorque_p2.txt"

t = []
torque = []

with open(filename, 'r') as t_file:
    file_lines = t_file.readlines()
    for current_line in file_lines:
        current_line = current_line.split("\t")
        t.append(float(current_line[0]))
        torque.append(float(current_line[1].strip()))

print()
findDeriv(t, torque)
print()

print("max torque {}, min torque {}".format(max(torque), min(torque)))
print("final time {}".format(t[-1]))


plt.plot(t, torque)
plt.xlabel("t (s)")
plt.ylabel("torque (Nm)")
plt.title("Torque v Time")
plt.show()
