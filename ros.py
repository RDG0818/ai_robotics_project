import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class TwistPublisher():
    def init(self):
        rospy.init_node("twist_pub")
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def move(self, cmd_x, cmd_z):
        msg = Twist()
        msg.linear.x = cmd_x
        msg.angular.z = cmd_z
        self.pub.publish(msg)
        rospy.loginfo("Published Twist: linear.x=%f angular.z=%f", cmd_x, cmd_z)

Bot = TwistPublisher()
rospy.sleep(2.0)
Bot.move(0.5, 0.0)
rospy.sleep(2.0)
Bot.move(0.0, 0.0)
rospy.spin()
