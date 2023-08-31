import rospy

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class TorqueInputNode:
    def __init__(self, namespace="prime"):
        print("TorqueInputNode: initializing node")

        self.namespace=namespace
        
        # Read data from file


        self.pose_gt_sub = rospy.Subscriber('/prime/pose_gt', Odometry, self.pose_callback)
        self.torque_msg Float32()
