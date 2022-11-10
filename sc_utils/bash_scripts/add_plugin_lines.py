sdf_orig_file = '/home/carson/sc_ws/src/spacecraft_sim/urdf/robots/polysat_perseus_description/urdf/polysat_perseus_working.sdf'
sdf_file = '/home/carson/sc_ws/src/spacecraft_sim/urdf/robots/polysat_perseus_description/urdf/polysat_perseus_robot.sdf'

# Number of parts to change
plume_thrusters = 14
plume_count = 0 
noz_thrusters = 14
noz_count = 0
sat_viz = 1
sat_count = 0
dyna_motors = 8
dyna_count = 0
link_count = 0

dyna_locs = list()
link_locs = list()

counter = 0
ii = 0

with open(sdf_orig_file, 'r') as world_opened:
    world_lines = world_opened.readlines()

line_num = len(world_lines)

while True:
    # Thrust plume visual plugin text
    thrust_vis_plug = '\t\t\t\t\t<plugin filename="libsc_thrust_visual_plugin.so" name="thrust_visual_'+str(plume_count)+'">\n\t\t\t\t\t\t<thrust_color>1 0 0 1</thrust_color>\n\t\t\t\t\t\t<thruster_number>'+str(plume_count)+'</thruster_number>\n\t\t\t\t\t</plugin>\n'
    # Thrust nozel color
    thrust_material = '\t\t\t\t\t<material>\n\t\t\t\t\t\t<ambient>0.1 0.1 0.1 1</ambient>\n\t\t\t\t\t\t<diffuse>0.1 0.1 0.2 1</diffuse>\n\t\t\t\t\t\t<specular>0 0 0 0</specular>\n\t\t\t\t\t\t<emissive>0 0 0 1</emissive>\n\t\t\t\t\t</material>\n'
    # Sat body color
    sat_material = '\t\t\t\t\t<material>\n\t\t\t\t\t\t<ambient>1.0 0.84 0 1</ambient>\n\t\t\t\t\t\t<diffuse>1.0 0.8 0 1</diffuse>\n\t\t\t\t\t\t<specular>1.0 0.8 0 1</specular>\n\t\t\t\t\t\t<emissive>0 0 0 1</emissive>\n\t\t\t\t\t</material>\n'
    # Dynamixel motor color
    dyna_material = '\t\t\t\t\t<material>\n\t\t\t\t\t\t<ambient>0.1 0.1 0.1 1</ambient>\n\t\t\t\t\t\t<diffuse>0.1 0.1 0.2 1</diffuse>\n\t\t\t\t\t\t<specular>0 0 0 0</specular>\n\t\t\t\t\t\t<emissive>0 0 0 1</emissive>\n\t\t\t\t\t</material>\n'
    # Arm link color
    link_material = '\t\t\t\t\t<material>\n\t\t\t\t\t\t<ambient>1 1 1 1</ambient>\n\t\t\t\t\t\t<diffuse>0.5 0.5 0.5 1</diffuse>\n\t\t\t\t\t\t<specular>0.1 0.1 0.1 0.1</specular>\n\t\t\t\t\t\t<emissive>0 0 0 1</emissive>\n\t\t\t\t\t</material>\n'

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
            id_location = ii + 1
            world_lines.insert(id_location, sat_material)
            sat_count += 1
            ii = 0
            line_num = len(world_lines)

        # Add motor colors
        if 'perseus' in world_lines[ii] and 'visual' in world_lines[ii] and 'Dynamixel' in world_lines[ii] and dyna_count < dyna_motors:
            if world_lines[ii] in dyna_locs:
                pass
            else:
                dyna_locs.append(world_lines[ii])
                print("On motor {}".format(dyna_count))
                id_location = ii + 1
                world_lines.insert(id_location, dyna_material)
                dyna_count += 1
                ii = 0
                line_num = len(world_lines)

        # Add link colors
        if 'perseus' in world_lines[ii] and 'visual' in world_lines[ii]:
            if 'Dynamixel' not in world_lines[ii]:
                if world_lines[ii] in link_locs:
                    pass
                else:
                    link_locs.append(world_lines[ii])
                    print("On link {}".format(link_count))
                    id_location = ii + 1
                    world_lines.insert(id_location, link_material)
                    link_count += 1
                    ii = 0
                    line_num = len(world_lines)
        ii += 1

    if ii == line_num:
        print("ii = line num")
        break

with open(sdf_file, 'w') as f:
    contents = "".join(world_lines)
    f.write(contents)

