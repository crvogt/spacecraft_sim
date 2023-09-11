import numpy as np
import rospy

from geometry_msgs.msg import PoseStamped

class PositionCommandNode:
    def __init__(self):
        print("PositionCommandNode: initializing node")

        self.cmd_val = PoseStamped()
        self.cmd_val.pose.position.x = 0.0 
        self.cmd_val.pose.position.y = 0.0 
        self.cmd_val.pose.position.z = 1
        self.cmd_val.pose.orientation.x = 0.0
        self.cmd_val.pose.orientation.y = 0.0
        self.cmd_val.pose.orientation.z = 0.0
        self.cmd_val.pose.orientation.w = 1.0

        self.cmd_pub = rospy.Publisher('cmd_pose', PoseStamped, queue_size=1)

        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            self.cmd_val.header.stamp = rospy.Time.now()
            self.cmd_pub.publish(self.cmd_val)
            rate.sleep()

if __name__=="__main__":
    print('Starting position_command.py')
    rospy.init_node('position_command')

    try:
        node = PositionCommandNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print('Caught an exception')
    print('exiting')
        
