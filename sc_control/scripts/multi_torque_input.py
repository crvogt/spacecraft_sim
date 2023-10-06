'''
Torque Profile
15.0198, 15.8076, 16.598
if t <= 15.0198, torque = -0.005
if t > 15.0198 and t <= 15.8076, torque = 0.005
if t > 15.8076 and t <= 16.598, torque = -0.005
if t > 16.598 and t <= 31.6153, torque = 0.005
if t > 31.6153, torque = 0
'''
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry

class TorqueInputNode:
    def __init__(self, namespace_a="robot1/prime_a", namespace_b="robot2/prime_b"):
        print("TorqueInputNode: initializing node")

        # self.namespace = namespace_a
        self.namespace_a = namespace_a
        self.namespace_b = namespace_b
        self.t_val = 0
        self.t_start = 0
        self.getPoseMsg = Odometry()
        self.torque_msg_a = Float32()
        self.torque_msg_b = Float32()
        self.pose_gt_sub = rospy.Subscriber("/prime_a/pose_gt", Odometry, self.pose_callback)
        self.torque_pub_a = rospy.Publisher('/%s/wheels/wheel_0'%(self.namespace_a), Float32, queue_size=1)
        self.torque_pub_b = rospy.Publisher('/%s/wheels/wheel_0'%(self.namespace_b), Float32, queue_size=1)
        self.t_list = [15.0198, 15.8076, 16.598, 31.6153]

    def pose_callback(self, _msg):
        self.getPoseMsg = _msg
        self.t_val = _msg.header.stamp.to_sec()
        if self.t_start == 0:
            self.t_start = self.t_val

        self.control_pose()

    def control_pose(self):
        te = self.t_val - self.t_start 
        # For a
        if te <= self.t_list[0]:
            self.torque_msg_a.data = -0.006
        elif te > self.t_list[0] and te <= self.t_list[1]:
            self.torque_msg_a.data = 0.005
        elif te > self.t_list[1] and te <= self.t_list[2]:
            self.torque_msg_a.data = -0.005
        elif te > self.t_list[2] and te <= self.t_list[3]:
            self.torque_msg_a.data = 0.006
        else:
            self.torque_msg_a.data = 0.0
        # For b 
        if te <= self.t_list[0]:
            self.torque_msg_b.data = 0.006
        elif te > self.t_list[0] and te <= self.t_list[1]:
            self.torque_msg_b.data = -0.005
        elif te > self.t_list[1] and te <= self.t_list[2]:
            self.torque_msg_b.data = 0.005
        elif te > self.t_list[2] and te <= self.t_list[3]:
            self.torque_msg_b.data = -0.006
        else:
            self.torque_msg_b.data = 0.0
        
        self.torque_pub_a.publish(self.torque_msg_a)
        self.torque_pub_b.publish(self.torque_msg_b)

if __name__=="__main__":
    print("Starting torque_input.py")
    rospy.init_node("torque_input")

    try:
        node = TorqueInputNode(namespace_a="robot1/prime_a", namespace_b="robot2/prime_b")
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Caught an exception")
    print("exiting")
        
