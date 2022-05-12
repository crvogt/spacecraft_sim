import os

import rospy
from sensor_msgs.msg import Joy

from std_msgs.msg import Float32

class ThrustTeleop:
    def __init__(self, namespace="polysat"):

        self.gain_dict = dict()
        # Forward/Back 0 1 2 3
        self.gain_dict[1] = [1.0, 1.0, -1.0, -1.0, 0, 0, 0, 0, 0, 0, 0, 0]
        # up 4 5
        self.gain_dict[5] = [0, 0, 0, 0, -0.5, -0.5, 0, 0, 0, 0, 0, 0]
        # down 6 7
        self.gain_dict[2] = [0, 0, 0, 0, 0, 0, -0.5, -0.5, 0, 0, 0, 0]
        # roll 4 5 6 7
        self.gain_dict[3] = [0, 0, 0, 0, -1.0, 1.0, 1.0, -1.0, 0, 0, 0, 0]
        # pitch 0 1 2 3
        self.gain_dict[4] = [1.0, -1.0, -1.0, 1.0, 0, 0, 0, 0, 0, 0, 0, 0]
        # yaw 8 9 10 11
        self.gain_dict[0] = [0, 0, 0, 0, 0, 0, 0, 0, -1.0, 1.0, 1.0, -1.0]

        self.num_thrusters = 12

        self.joy2thrust = [rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 0), Float32, queue_size=1),
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
                           rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 11), Float32, queue_size=1)]

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def joy_callback(self, joy):
        msg = Float32()
        aa_dict = dict()

        for aa, ii in zip(joy.axes, range(len(joy.axes))):
            aa_dict[ii] = aa

        key_list = list(self.gain_dict.keys())
        msg_values = [0] * self.num_thrusters

        for key_val in key_list:
            for ii in range(len(self.gain_dict[key_val])):
                msg_values[ii] += aa_dict[key_val] * self.gain_dict[key_val][ii]
       
        # TODO Check this loop
        for ii in range(len(self.joy2thrust)):
            msg.data = msg_values[ii]
            self.joy2thrust[ii].publish(msg)

if __name__ == '__main__':
    # Start the node 
    node_name = os.path.splitext(os.path.basename(__file__))[0]
    rospy.init_node(node_name)
    rospy.loginfo('Starting [%s] node' % node_name)

    # Get params
    ns = 'polysat'
    if rospy.has_param('~namespace'):
        ns = rospy.get_param('~namespace')

    teleop = ThrustTeleop(namespace = ns)
    rospy.spin()
    rospy.loginfo('Shutting down [%s] node' % node_name)
