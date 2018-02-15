from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata

import rospy
import time

COMMAND_PERIOD = 100 #ms

class Drone():
    def __init__(self, navdataListener):
        rospy.init_node('ardrone_flight', anonymous=False)
        self.rate = rospy.Rate(15)
        self.pubTakeoff = rospy.Publisher("ardrone/takeoff",Empty, queue_size=100)
        self.pubLand = rospy.Publisher("ardrone/land",Empty, queue_size=100)
        self.pubCommand = rospy.Publisher('cmd_vel',Twist, queue_size=100)
        self.command = Twist()
        self.navdata = rospy.Subscriber("ardrone/navdata", Navdata, navdataListener, queue_size=100)
        #self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0), self.Command)
        rospy.on_shutdown(self.land)


    def takeOff(self):
        print('sending take off')
        time.sleep(3) # time to initialize
        self.pubTakeoff.publish(Empty())

        time.sleep(5) # time to execute taking off
        print('take off is executed\n')


    def land(self):
        print('sending land')
        self.pubLand.publish(Empty())
        self.rate.sleep()
        print('land is executed\n')
        

    def stop(self):
        print('sending stop')
        self.Command(0, 0, 0, 0, 0, 0)
        print('stop is executed\n')


    def forward(self, speed):
        print('sending forward with speed:{}'.format(speed))
        self.Command(speed, 0, 0, 0, 0, 0)
        print('forward is executed\n')


    def right(self, speed):
        print('sending right with speed:{}'.format(speed))
        self.Command(0, -speed, 0, 0, 0, 0)
        print('right command is executed\n')


    def up(self, speed):
        print('sending up with speed:{}'.format(speed))
        self.Command(0, 0, speed, 0, 0, 0)
        print('up is executed\n')


    def clockwise(self, speed):
        print('sending clockwise with speed:{}'.format(speed))
        self.Command(0, 0, 0, 0, 0, -speed)
        print('clockwise is executed\n')


    def Command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pubCommand.publish(self.command)
        self.rate.sleep()


