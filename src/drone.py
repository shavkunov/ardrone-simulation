from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata

import rospy

class Drone():
    def __init__(self, navdataListener):
        rospy.init_node('ardrone_flight', anonymous=False)
        self.rate = rospy.Rate(10)
        self.pubTakeoff = rospy.Publisher("ardrone/takeoff",Empty, queue_size=10)
        self.pubLand = rospy.Publisher("ardrone/land",Empty, queue_size=10)
        self.pubCommand = rospy.Publisher('cmd_vel',Twist, queue_size=10)
        self.command = Twist()
        self.navdata = rospy.Subscriber("ardrone/navdata", Navdata, navdataListener)
        self.state_change_time = rospy.Time.now()
        rospy.on_shutdown(self.land)

    def navdataCallback(self, navdata):
        time = navdata.header.stamp.to_sec()
        # navigation data proccessing

    def takeOff(self):
        self.pubTakeoff.publish(Empty())
        self.rate.sleep()

    def land(self):
        self.pubLand.publish(Empty())

    def stop(self):
        self.Command(0, 0, 0, 0, 0, 0)

    def forward(self, speed):
        self.Command(speed, 0, 0, 0, 0, 0)

    def right(self, speed):
        self.Command(0, speed, 0, 0, 0, 0)

    def up(self, speed):
        self.Command(0, 0, speed, 0, 0, 0)

    def clockwise(self, speed):
        self.Command(0, 0, 0, 0, 0, speed)

    def Command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pubCommand.publish(self.command)
        self.rate.sleep()
