sdf_orig_file = '/home/carson/sc_ws/src/spacecraft_sim/urdf/robots/polysat_description/urdf/polysat_robot_world.sdf'
sdf_file = '/home/carson/sc_ws/src/spacecraft_sim/urdf/robots/polysat_description/urdf/polysat_robot.sdf'

# Number of parts to change
plume_thrusters = 14
plume_count = 0 
noz_thrusters = 14
noz_count = 0
sat_viz = 1
sat_count = 0

counter = 0
ii = 0

with open(sdf_orig_file, 'r') as world_opened:
    world_lines = world_opened.readlines()

line_num = len(world_lines)

while True:
    # Thrust plume visual plugin text
    thrust_vis_plug = '\t\t\t\t\t<plugin filename="libsc_thrust_visual_plugin.so" name="thrust_visual_'+str(counter)+'">\n\t\t\t\t\t\t<thrust_color>1 0 0 1</thrust_color>\n\t\t\t\t\t\t<thruster_number>'+str(counter)+'</thruster_number>\n\t\t\t\t\t</plugin>\n'
    # Thrust nozel color
    thrust_material = '\t\t\t\t\t<material>\n\t\t\t\t\t\t<ambient>0.1 0.1 0.1 1</ambient>\n\t\t\t\t\t\t<diffuse>0.1 0.1 0.2 1</diffuse>\n\t\t\t\t\t\t<specular>0 0 0 0</specular>\n\t\t\t\t\t\t<emissive>0 0 0 1</emissive>\n\t\t\t\t\t</material>\n'
    # Sat body color
    sat_material = '\t\t\t\t\t<material>\n\t\t\t\t\t\t<ambient>0.1 0.1 0.1 1</ambient>\n\t\t\t\t\t\t<diffuse>0.1 0.1 0.2 1</diffuse>\n\t\t\t\t\t\t<specular>0 0 0 0</specular>\n\t\t\t\t\t\t<emissive>0 0 0 1</emissive>\n\t\t\t\t\t</material>\n'

    # Go through each line and check for the thrust plume visual
    while ii < line_num:
        check_str = 'thrust_plume_' + str(plume_count) + '_visual_visual'
        if check_str in world_lines[ii] and (plume_count < plume_thrusters):
            # Print which thruster we're on
            print("On plume count " + str(plume_count))
            id_location = ii + 2
            world_lines.insert(id_location, thrust_vis_plug)
            plume_count += 1
            ii = 0
            line_num = len(world_lines)

        check_str = 'thruster_' + str(noz_count) + '_visual_visual'
        if check_str in world_lines[ii] and (noz_count < noz_thrusters):
            # Print which thruster we're on
            print("On thruster count " + str(noz_count))
            id_location = ii + 8
            world_lines.insert(id_location, thrust_material)
            noz_count += 1
            ii = 0
            line_num = len(world_lines)
        
        check_str = 'polysat_link_visual'
        if check_str in world_lines[ii] and (sat_count < sat_viz):
            # Print which thruster we're on
            print("On sat count " + str(sat_count))
            id_location = ii + 8
            world_lines.insert(id_location, sat_material)
            sat_count += 1
            ii = 0
            line_num = len(world_lines)

        ii += 1

    if ii == line_num:
        print("ii = line num")
        break

with open(sdf_file, 'w') as f:
    contents = "".join(world_lines)
    f.write(contents)

