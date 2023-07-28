import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance

def broadcast_tf(msg):
    orientation = msg.pose.pose.orientation
    position = msg.pose.pose.position
    br = tf.TransformBroadcaster()
    br.sendTransform((position.x, position.y, position.z),
                     (orientation.x, orientation.y, orientation.z,
                      orientation.w),
                     rospy.Time.now(),
                     "polysat/base_link",
                     "world")

if __name__ == '__main__':
    rospy.init_node("robot_tf_broadcaster")
    rospy.Subscriber("/polysat/pose_gt",
                     Odometry,
                     broadcast_tf)

    rospy.spin()

