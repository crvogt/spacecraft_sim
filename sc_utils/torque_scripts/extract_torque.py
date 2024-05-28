import matplotlib.pyplot as plt
import numpy as np


def findDeriv(time, values):
    prev_val = 0
    cur_val = 0
    prev_time = 0
    cur_time = 0
    deriv_list = []
    t_list = []

    print()
    
    for ii, val in enumerate(values):
        if ii == 0:
            prev_val = val
            prev_time = time[ii]
        else:
            if(time[ii] != prev_time):
                deriv = (val - prev_val)/(time[ii] - prev_time)
                print(deriv)
                if abs(deriv) > 0:
                    # print("{} {}".format(prev_time, prev_val))
                    print("{:.10f},".format(prev_time), end="")
                    deriv_list.append(prev_val)
                    t_list.append(prev_time)
                prev_time = time[ii]
                prev_val = val
    
    print("\n",end="\n")

    return t_list, deriv_list


filename = "../data/paper_files/Paper1_Ex1/myFile_ResttoRest_time_torque.txt"
#filename = "../data/paper_files/Paper1_Ex2/myFile_nonRest_to_Rest_time_torque.txt"

t = []
torque = []

with open(filename, 'r') as t_file:
    file_lines = t_file.readlines()
    for current_line in file_lines:
        current_line = current_line.split("\t")
        t.append(float(current_line[0]))
        torque.append(float(current_line[1].strip()))

ts_list, derivs_list = findDeriv(t, torque)

print("max torque {}, min torque {}".format(max(torque), min(torque)))
print("final time {}".format(t[-1]))

plt.plot(t, torque)
plt.scatter(ts_list, derivs_list)
plt.xlabel("t (s)")
plt.ylabel("torque (Nm)")
plt.title("Torque v Time")
plt.xlim([0, t[-1]])
plt.show()
