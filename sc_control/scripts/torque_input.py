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
    def __init__(self, namespace="prime"):
        print("TorqueInputNode: initializing node")

        self.namespace = namespace
        self.t_val = 0
        self.t_start = 0
        self.getPoseMsg = Odometry()
        self.torque_msg = Float32()
        self.pose_gt_sub = rospy.Subscriber("/%s/pose_gt"%(self.namespace), Odometry, self.pose_callback)
        self.torque_pub = rospy.Publisher('/%s/wheels/wheel_0'%(self.namespace), Float32, queue_size=1)
        #self.t_list = [15.0198, 15.8076, 16.598, 31.6153]
        self.t_list = [17.2093, 20.11859, 21.1105, 34.43167]

    def pose_callback(self, _msg):
        self.getPoseMsg = _msg
        self.t_val = _msg.header.stamp.to_sec()
        if self.t_start == 0:
            self.t_start = self.t_val

        self.control_pose()

    def control_pose(self):
        te = self.t_val - self.t_start 
        if te <= self.t_list[0]:
            self.torque_msg.data = -0.005
        elif te > self.t_list[0] and te <= self.t_list[1]:
            self.torque_msg.data = 0.005
        elif te > self.t_list[1] and te <= self.t_list[2]:
            self.torque_msg.data = -0.005
        elif te > self.t_list[2] and te <= self.t_list[3]:
            self.torque_msg.data = 0.005
        else:
            self.torque_msg.data = 0.0

        print("Publishing torque {}, te {}".format(self.torque_msg.data, te))
        self.torque_pub.publish(self.torque_msg)

if __name__=="__main__":
    print("Starting torque_input.py")
    rospy.init_node("torque_input")

    try:
        node = TorqueInputNode(namespace="prime")
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Caught an exception")
    print("exiting")
        
