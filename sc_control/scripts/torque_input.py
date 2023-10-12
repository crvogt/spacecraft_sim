'''
Torque Profile
15.0198, 15.8076, 16.598
if t <= 15.0198, torque = -0.005
if t > 15.0198 and t <= 15.8076, torque = 0.005
if t > 15.8076 and t <= 16.598, torque = -0.005
if t > 16.598 and t <= 31.6153, torque = 0.005
if t > 31.6153, torque = 0
'''
'''
1.0, -0.000827894438004
2.0, -0.001295565135116
3.0, -0.000838169041804
4.0, -0.000087827427775846
5.0, -0.000186558135836
6.0, -0.000197603402095
7.0, -0.0000029183942540684
8.0, -0.00000392627529342669
9.0, -0.0000354936317478965
10.0, 0.00000822341684026002
11.0, 0.0000173509835180966
12.0, 0.00000420885021810585
13.0, 0.0000119411245221161
14.0, 0.0000168791516727308
15.0, 0.0000130468426564513
16.0, 0.0000137161317089084
17.0, 0.0000154958384761815
18.0, 0.0000146131082817593
19.0, 0.0000143644714958696
20.0, 0.0000148449171162525
21.0, 0.0000146695790745876
22.0, 0.000014467898057828
23.0, 0.0000145371511386348
24.0, 0.0000144822865190691
25.0, 0.000014368544515913
26.0, 0.000014328363314278
27.0, 0.0000142762939665455
28.0, 0.0000142014644058827
29.0, 0.0000141426408963211
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
        self.ii = 0
        self.t_list = [1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0,11.0,12.0,
                       13.0,14.0,15.0,16.0,17.0,18.0,19.0,20.0,21.0,22.0,23.0,24.0,25.0,
                       26.0,27.0,28.0,29.0]
        self.tq_list = [-0.0008278944,-0.0012955651,-0.0008381690,-0.0000878274,-0.0001865581,
                        -0.0001976034,-0.0000029184,-0.0000039263,-0.0000354936,0.0000082234,
                        0.0000173510,0.0000042089,0.0000119411,0.0000168792,0.0000130468,
                        0.0000137161,0.0000154958,0.0000146131,0.0000143645,0.0000148449,
                        0.0000146696,0.0000144679,0.0000145372,0.0000144823,0.0000143685,
                        0.0000143284,0.0000142763,0.0000142015,0.0000141426]


    def pose_callback(self, _msg):
        self.getPoseMsg = _msg
        self.t_val = _msg.header.stamp.to_sec()
        if self.t_start == 0:
            self.t_start = self.t_val

        self.control_pose()

    def control_pose(self):
        te = self.t_val - self.t_start
        if te <= self.t_list[self.ii]:
            self.torque_msg.data = self.tq_list[self.ii] 
        elif (te > self.t_list[self.ii]):
            if te > self.t_list[-1]:
                self.torque_msg.data = 0.0
            else:
                self.ii += 1
                self.torque_msg.data = self.tq_list[self.ii]
        

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
        
