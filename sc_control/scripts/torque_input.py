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
        self.t_list = [2.0000000000,2.5000000000,3.0000000000,3.5000000000,4.0000000000,
                       4.5000000000,5.0000000000,5.5000000000,6.0000000000,6.5000000000,
                       7.0000000000,7.5000000000,8.0000000000,8.5000000000,9.0000000000,
                       9.5000000000,10.0000000000,10.5000000000,11.0000000000,11.5000000000,
                       12.0000000000,12.5000000000,13.0000000000,13.5000000000,14.0000000000,
                       14.5000000000,15.0000000000,15.5000000000,16.0000000000,16.5000000000,
                       17.0000000000,17.5000000000,18.0000000000,18.5000000000,19.0000000000,
                       19.5000000000,20.0000000000,20.5000000000,21.0000000000,21.5000000000,
                       22.0000000000,22.5000000000,23.0000000000,23.5000000000,24.0000000000,
                       24.5000000000,25.0000000000,25.5000000000,26.0000000000,26.5000000000,
                       27.0000000000,27.5000000000,28.0000000000,28.5000000000,29.0000000000,29.5000000000]
        self.tq_list = [-0.0500000000,-0.0243386435,0.0273162041,0.0144332013,0.0081861634,0.0172224477,
                        0.0145287120,0.0070480608,0.0108536280,0.0123978514,0.0069817403,0.0070323195,
                        0.0094508531,0.0067351387,0.0050740776,0.0067301980,0.0059818870,0.0041315045,
                        0.0046979479,0.0048946189,0.0035738878,0.0033729810,0.0037647998,0.0030798815,
                        0.0025649654,0.0027984530,0.0025617719,0.0020561908,0.0020704052,0.0020464232,
                        0.0016901343,0.0015612323,0.0015847977,0.0013861119,0.0012118483,0.0012084401,
                        0.0011174375,0.0009631420,0.0009216735,0.0008830703,0.0007734644,0.0007107897,
                        0.0006875154,0.0006199573,0.0005560071,0.0005317260,0.0004926088,0.0004396152,
                        0.0004117392,0.0003876045,0.0003490747,0.0003207554,0.0003028574,0.0002767765,
                        0.0002516144,0.0002359916]


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
        
