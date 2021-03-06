sdf_file = '/home/carson/spaceship_ws/src/spacecraft_sim/urdf/robots/polysat_description/urdf/polysat_robot.sdf'

num_thrusters = 14
plume_location = 10 
counter = 10
ii = 0
while counter < num_thrusters:
    print("On count " + str(counter))
    world_opened = open(sdf_file, 'r')
    world_lines = world_opened.readlines()

    line_num = len(world_lines)

    while ii < line_num:
        check_str = 'thrust_plume_' + str(counter) + '_visual_visual'
        if check_str in world_lines[ii]:
            plume_location = ii + 2
            break
        ii += 1

    print(plume_location)
    print(check_str)
    if plume_location == 0:
        print("We have a location issue...")

    world_opened.close()

    thrust_values = '\t\t\t\t\t<plugin filename="libsc_thrust_visual_plugin.so" name="thrust_visual_'+str(counter)+'">\n\t\t\t\t\t\t<thrust_color>1 0 0 1</thrust_color>\n\t\t\t\t\t\t<thruster_number>'+str(counter)+'</thruster_number>\n\t\t\t\t\t</plugin>\n'
    world_lines.insert(plume_location, thrust_values)

    with open(sdf_file, 'w') as f:
        contents = "".join(world_lines)
        f.write(contents)

    counter += 1
