import os

import rospy
from sensor_msgs.msg import Joy

from std_msgs.msg import Float32

class ThrustTeleop:
    def __init__(self, namespace="polysat"):

        self.gain_dict = dict()
        # left/right
        self.gain_dict[0] = [1.0, 1.0, -1.0, -1.0]
        # up/down
        self.gain_dict[1] = [1.0, 1.0, -1.0, -1.0]

        self.joy2thrust = dict()

        # left/right
        self.joy2thrust[0] = [rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 0), Float32, queue_size=1),
                              rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 1), Float32, queue_size=1),
                              rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 2), Float32, queue_size=1),
                              rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 3), Float32, queue_size=1)]

        # up/down 
        self.joy2thrust[1] = [rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 4), Float32, queue_size=1),
                              rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 5), Float32, queue_size=1),
                              rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 6), Float32, queue_size=1),
                              rospy.Publisher('/%s/thrusters/thruster_%d/'%(namespace, 7), Float32, queue_size=1)]
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def joy_callback(self, joy):
        msg = Float32()
        aa_dict = dict()

        for aa, ii in zip(joy.axes, range(len(joy.axes))):
            aa_dict[ii] = aa

        key_list = list(self.joy2thrust.keys())

        for key_val in key_list:
            # Keep vertical thrusters
            for ii in range(len(self.gain_dict[key_val])):
                msg.data = aa_dict[key_val] * self.gain_dict[key_val][ii]
                self.joy2thrust[key_val][ii].publish(msg)

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
