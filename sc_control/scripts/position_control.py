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
        
        # Set PID values
        # Simple split
        # R, P, Y, x, y, z
        self.p_vals = np.array([])
        self.i_vals = np.array([])
        self.d_vals = np.array([])
        self.sat_vals = np.array([])


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


if __name__=="__main__":
    print('Starting PositionControl.py')
    rospy.init_node('position_control')

    try:
        node = PositionControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Caught an exception')
    print('exciting')
        
