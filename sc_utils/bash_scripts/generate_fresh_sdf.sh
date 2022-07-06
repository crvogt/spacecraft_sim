cd /home/carson/spaceship_ws/src/spacecraft_sim/urdf/robots/polysat_description/urdf

USER=carson

XACRO_FILE=polysat_robot.urdf.xacro
URDF_FILE=polysat_robot.urdf
SDF_FILE=polysat_robot.sdf
SDF_TMP=polysat_robot_tmp.sdf

# Check if URDF or SDF file exists. If they do, delete them
if [ -f "$URDF_FILE" ]; then
  echo "$URDF_FILE exists, deleting..."
  rm $URDF_FILE
fi
if [ -f "$SDF_FILE" ]; then
  echo "$SDF_FILE exists, deleting..."
  rm $SDF_FILE
fi

echo "Generating $URDF_FILE..."
xacro $XACRO_FILE > $URDF_FILE 
chmod 664 $URDF_FILE

echo "Generating $SDF_FILE..."
gz sdf -p $URDF_FILE > $SDF_FILE
chmod 664 $SDF_FILE

# This is dumb. Learn "sed -i"
echo "Adding plugin text to $SDF_FILE..."
awk '/polysat_link_visual/ { print; print "\t\t\t\t<plugin filename=\
 \"libsc_thrust_visual_plugin.so\" name=\"thrust_visual\">\
 \n\t\t\t\t\t<thrust_color>1 0 0 1</thrust_color>\
 \n\t\t\t\t</plugin>"; next }1' $SDF_FILE >> $SDF_TMP

rm $SDF_FILE; mv $SDF_TMP ./$SDF_FILE
chmod 664 $SDF_FILE
