sdf_file = '/home/carson/spaceship_ws/src/spacecraft_sim/urdf/robots/polysat_description/urdf/polysat_robot.sdf'

sdf_opened = open(sdf_file, 'r')
sdf_lines = sdf_opened.readlines()

num_thrusters = 14
plume_location = [8] * 14
counter = 0
ii = 0

line_num = len(sdf_lines)

while ii < line_num:
    check_str = 'thrust_plume_' + str(counter) + '_visual_visual'
    if check_str in sdf_lines[ii]:
        plume_location[counter] = ii + 2
        counter += 1
        ii = 0
    ii += 1

sdf_opened.close()


for ii, idx in enumerate(plume_location):
    thrust_values = '\t\t\t\t<plugin filename="libsc_thrust_visual_plugin.so" name="thrust_visual_'+str(ii)+'">\n\t\t\t\t\t<thrust_color>1 0 0 1</thrust_color>\n\t\t\t\t\t<thruster_number>'+str(ii)+'</thruster_number>\n\t\t\t\t</plugin>\n'
    sdf_lines.insert(idx, thrust_values)


with open(sdf_file, 'w') as f:
    contents = "".join(sdf_lines)
    f.write(contents)
