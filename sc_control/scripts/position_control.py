import numpy as np
import rospy
import tf.transformations as trans

from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Imu
import geometry_msgs.msg as geometry_msgs
from PID import PIDControllerBase 

class PositionControllerNode:
    def __init__(self):
        print("PositionControllerNode: initializing node")

        self.pos_des = np.zeros(3)
        self.quat_des = np.array([0, 0, 0, 1])

        self.initialized = False

        # Initialize pids with default parameters
        self.pid_rot = PIDControllerBase(2, 0, 0, 1)
        # self.pid_pos = pid_controller_base.PIDControllerBase(1, 0, 0, 1)

        # ROS
        #self.sub_cmd_pose = rospy.Subscriber('cmd_pose', numpy_msg(geometry_msgs.PoseStamped), self.cmd_pose_callback)
        self.position_sub = rospy.Subscriber('polysat/imu', Imu, self.odometry_callback)

    def cmd_pose_callback(self, _msg):
        # Store the desired pose
        p = msg.pose.position
        q = msg.pose.orientation
        self.pos_des = np.array([p.x, p.y, p.z])
        self.quat_des = np.array([q.x, q.y, q.z, q.w])

    def odometry_callback(self, _msg):
        print(_msg)

if __name__=="__main__":
    print('Starting PositionControl.py')
    rospy.init_node('position_control')

    try:
        node = PositionControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Caught an exception')
    print('exciting')
        
