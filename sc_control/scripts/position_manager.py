import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

'''
With this node, we'll subscribe to pose_gt and publish a position
cmd under cmd_pose

We can set a point and have the node divide that into waypoints to not
exceed our speed limit
'''

class PositionManagerNode:
    def __init__(self):
        print("PositionManagerNode: initializing node")

        self.cmd_val = PoseStamped()

        self.cmd_pub = rospy.Publisher('cmd_pose', PoseStamped, queue_size=1)
        self.pose_gt_sub = rospy.Subscriber('/polysat/pose_gt', Odometry, self.pose_callback)

        rate = rospy.Rate(50)
        while note rospy.is_shutdown():
            self.cmd_val.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(self.cmd_val)
            rate.sleep()

    def pose_callback(self, _msg):
        self.getPoseMsg = _msg
        self.

if __name__=="__main__":
    print("Starting position_manager.py")
    rospy.init_node("position_manager")

    try: 
        node = PositionManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Caugght an exception")
    print("exiting")
