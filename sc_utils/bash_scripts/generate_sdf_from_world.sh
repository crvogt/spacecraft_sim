cd /home/carson/spaceship_ws/src/spacecraft_sim/urdf/robots/polysat_description/urdf

USER=carson

SDF_FILE=polysat_robot.sdf
SDF_TMP=polysat_robot_tmp.sdf

# Check if URDF or SDF file exists. If they do, delete them
if [ -f "$SDF_FILE" ]; then
  echo "$SDF_FILE exists..."
fi

# This is dumb. Learn "sed -i"
echo "Adding plugin text to $SDF_FILE..."
for i in {0..13}
do
echo $i
STRING_A='\t\t\t\t<plugin filename="libsc_thrust_visual_plugin.so" name="thrust_visual_'$i'">\n\t\t\t\t\t<thrust_color>1 0 0 1</thrust_color>\n\t\t\t\t\t<thruster_number>'$i'</thruster_number>\n\t\t\t\t</plugin>'
SEARCH_VAR='thrust_plume_'$i'_visual_visual'
sed -i 's/thrust_plume_0/{p;STRING_A}' $SDF_FILE
#rm $SDF_FILE
 #mv $SDF_TMP ./$SDF_FILE
done

# chmod 664 $SDF_FILE
