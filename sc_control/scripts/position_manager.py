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

        self.is_initialized = False
        self.cmd_val = PoseStamped()

        self.cmd_pub = rospy.Publisher('cmd_pose', PoseStamped, queue_size=1)
        self.pose_gt_sub = rospy.Subscriber('/polysat/pose_gt', Odometry, self.pose_callback)

        self.waypoint_list = [(2.5, 0, 5, 0, 0, 0, 1),
                (2.5, 0, 5, 0, 0, 0, 1), 
                (3.0, 0, 6, 0, 0, 0, 1),
                (3.0, 0, 5.5, 0, 0, 0, 1),
                (3.0, 0, 5, 0, 0, 0, 1),
                (3.0, 0, 4.5, 0, 0, 0, 1),
                (3.0, 0, 4, 0, 0, 0, 1),
                (2.0, 0, 3.5, 0, 0, 0, 1),
                (2.0, 1, 3.5, 0, 0, 0, 1),
                (2.0, 2, 3.5, 0, 0, 0, 1),
                (2.0, 2, 3.5, 0, 0, 0.407, 0.907),
                (2.0, 2, 3.5, 0, 0, 0, 1),
                (2.5, 2, 4.0, 0, 0, 0, 1),
                (3.0, 2, 5.0, 0, 0, 0, 1),
                (3.0, 2.35, 5.0, 0, 0, 0, 1)]

                #(2.75, -1, 5, 0, 0, 0, 1), 
                #(2.75, -1, 5, 0, 0, 0.195, 0.981), 
                #(3, -2, 5, 0, 0, 0.195, 0.981),
                #(3, -2, 5, 0, 0, 0.383, 0.924),
                #(3.75, -2.5, 5, 0, 0, 0.383, 0.924),  
                #(3.75, -2.5, 5, 0, 0, 0.556, 0.831),  
                #(4.5, -3.5, 5, 0, 0, 0.556, 0.831),
                #(4.5, -3.5, 5, 0, 0, 0.6, 0.77),
                #(5, -4, 5, 0, 0, 0.607, 0.77), 
                #(5, -4, 5, 0, 0, 0.707, 0.707), 
                #(5.5, -3, 5, 0, 0, 0.831, 0.556), 
                #(6, -2, 5, 0, 0, 0.924, 0.383), 
                #(7, 0, 5, 0, 0, 1, 0), 
                #(6, 2, 5, 0, 0, 0.981, -0.195), 
                #(5.5, 3, 5, 0, 0, 0.924, -0.383), 
                #(5, 4, 5, 0, 0, 0.831, -0.556),
                #(4, 3, 5, 0, 0, 0.707, -0.707), 
                #(3, 2, 5, 0, 0, 0.556, -0.831),
                #(2.75, 1, 5, 0, 0, 0.383, -0.924), 
                #(2.5, 0, 5, 0, 0, 1)] 
        self.waypoint_num = 0

    def pose_callback(self, _msg):
        self.getPoseMsg = _msg
        
        if self.pose_dist() < 0.1 and self.waypoint_num < (len(self.waypoint_list)-1):
            counter = 0
            while counter < 100000000:
                counter += 1
            self.waypoint_num += 1
        self.publish_cmd_pose(self.waypoint_list[self.waypoint_num])

    def publish_cmd_pose(self, _waypoint):
        self.cmd_val.header.stamp = rospy.Time.now()
        self.cmd_val.pose.position.x = _waypoint[0]
        self.cmd_val.pose.position.y = _waypoint[1]
        self.cmd_val.pose.position.z = _waypoint[2]
        self.cmd_val.pose.orientation.x = _waypoint[3] 
        self.cmd_val.pose.orientation.y = _waypoint[4]
        self.cmd_val.pose.orientation.z = _waypoint[5]
        self.cmd_val.pose.orientation.w = _waypoint[6]
        
        self.cmd_pub.publish(self.cmd_val)

    def pose_dist(self):
        waypoint = self.waypoint_list[self.waypoint_num]
        cur_loc = [self.getPoseMsg.pose.pose.position.x,
                        self.getPoseMsg.pose.pose.position.y,
                        self.getPoseMsg.pose.pose.position.z]

        return np.sqrt((cur_loc[0]-waypoint[0])**2 + 
               (cur_loc[1]-waypoint[1])**2 + 
               (cur_loc[0]-waypoint[0])**2)


if __name__=="__main__":
    print("Starting position_manager.py")
    rospy.init_node("position_manager")

    try: 
        node = PositionManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Caugght an exception")
    print("exiting")
