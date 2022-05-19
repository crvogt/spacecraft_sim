import numpy as np
import rospy
import tf.transformations as trans

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped 
from PID import PIDControllerBase 

from gazebo_msgs.msg import ModelStates

class PositionControllerNode:
    def __init__(self, namespace="polysat"):
        print("PositionControllerNode: initializing node")

        # Set information variables
        self.getStateMsg = ModelStates()
        self.getImuMsg = Imu()
        self.getPosCmd = PoseStamped() 
        self.poseVal = PoseStamped()
        self.t_val = 0
        
        # Set PID values
        # Simple split
        # R, P, Y, x, y, z
        self.p_vals = np.array([0, 0, 0, 0, 0, 1])
        self.i_vals = np.array([0, 0, 0, 0, 0, 1])
        self.d_vals = np.array([0, 0, 0, 0, 0, 1])
        self.sat_vals = np.array([0, 0, 0, 0, 0, 1])
        self.pid_list = [0] * 6 
        self.set_pid_vals()

        # Set flags
        self.initialized = False
        # Set this to false till we can determine 3D pose
        self.use_imu = False

        # ROS
        self.sub_cmd_pose = rospy.Subscriber('cmd_pose', PoseStamped, self.cmd_pose_callback)
        self.position_sub = rospy.Subscriber('polysat/imu', Imu, self.odometry_callback)
        self.model_state_sub = rospy.Subscriber('gazebo/model_states', ModelStates, self.state_callback)
        # Add thruster subs
        self.thruster_pubs()

    def cmd_pose_callback(self, _msg):
        # Store the desired pose
        self.getPosCmd = _msg
        self.t_val = _msg.header.stamp.to_sec()

    def odometry_callback(self, _msg):
        self.getImuMsg = _msg
        if self.use_imu:
            self.control_pose()

    def state_callback(self, _msg):
        self.getStateMsg = _msg
        if not self.use_imu:
            self.poseVal.pose = self.getStateMsg.pose[-1]
            self.control_pose()

    def control_pose(self):
        # Need to map output to each thruster
        self.getPosCmd
        self.poseVal

        # Angular error
        cmd_angles = trans.euler_from_quaternion(self.getPosCmd.pose.quaternion)
        cur_angles = trans.euler_from_quaternion(self.poseVal.pose.quaternion)
        Rol_err = cmd_angles[0] - cur_angles[0] 
        Pit_err = cmd_angles[1] - cur_angles[1] 
        Yaw_err = cmd_angles[2] - cur_angles[2] 

        # Spatial error
        cmd_pose = self.getPosCmd.pose.position
        cur_pose = self.poseVal.pos.position
        x_err = cmd_pose.x - cur_pose.x 
        y_err = cmd_pose.y - cur_pose.y  
        z_err = cmd_pose.z - cur_pose.z  

        error_list = [Rol_err, Pit_err, Yaw_err,
                      x_err, y_err, z_err]
        # Send values to regulator


    def thruster_pubs(self):
        self.thrust_list = [rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 0), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 1), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 2), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 3), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 4), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 5), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 6), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 7), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 8), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 9), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 10), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 11), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 12), Float32, queue_size=1),
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 13), Float32, queue_size=1)]

    def set_pid_vals(self):
        for ii in range(6):
            self.pid_list[ii] = PIDRegulator(self.p_vals[ii],
                                             self.i_vals[ii],
                                             self.d_vals[ii],
                                             self.sat_vals[ii])


if __name__=="__main__":
    print('Starting PositionControl.py')
    rospy.init_node('position_control')

    try:
        node = PositionControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Caught an exception')
    print('exciting')
        
