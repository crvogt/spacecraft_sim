import time

# File that needs paths changed. This one is enough to spawn the model in 
# polysat_sdf.launch

filename = "/home/carson/sc_ws/src/spacecraft_sim/urdf/robots/polysat_perseus_description/urdf/polysat_perseus_robot.sdf"

check_a = 'polysat_description'
#check_a = 'perseus_description'
path_sub_a = '<uri>model://' + check_a

print("Modifying file {}...".format(filename))
# Give time to back out...
time.sleep(2)
with open (filename, 'r') as f:
    file_lines = f.readlines()

line_num = len(file_lines)

ii = 0
lines_changed = 0
# Go through and check each line for characters matching "check_a"
while ii < line_num:
    if check_a in file_lines[ii]:
        line_split = file_lines[ii].split(check_a)
        new_str = "\t\t\t\t\t\t\t" + path_sub_a + line_split[1]  
        # Swap them out here
        file_lines[ii] = new_str
        lines_changed += 1
    ii += 1
print("Changing {} lines...".format(lines_changed))
time.sleep(2)

print("Writing to new file in 2 seconds...")
time.sleep(2)
# Write all of our modified lines back to the file
with open(filename, 'w') as f:
    contents = "".join(file_lines)
    f.write(contents)
