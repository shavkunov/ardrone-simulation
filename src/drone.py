from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata

import rospy
import time
import logging

COMMAND_PERIOD = 100 #ms

class Drone():
    def __init__(self, navdataListener):
        logging.basicConfig(filename='logs',level=logging.DEBUG)
        rospy.init_node('ardrone_flight', anonymous=False)
        self.rate = rospy.Rate(10)
        self.pubTakeoff = rospy.Publisher("ardrone/takeoff",Empty, queue_size=10)
        self.pubLand = rospy.Publisher("ardrone/land",Empty, queue_size=10)
        self.pubCommand = rospy.Publisher('cmd_vel',Twist, queue_size=10)
        self.command = Twist()
        self.navdata = rospy.Subscriber("ardrone/navdata", Navdata, navdataListener)
        #self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0), self.Command)
        rospy.on_shutdown(self.land)

    def takeOff(self):
        logging.debug('sending take off')
        time.sleep(3) # time to initialize
        self.pubTakeoff.publish(Empty())

        time.sleep(3) # time to execute taking off
        self.rate.sleep()
        logging.debug('take off is executed')

    def land(self):
        logging.debug('sending land')
        self.pubLand.publish(Empty())
        self.rate.sleep()
        logging.debug('land is executed')
        

    def stop(self):
        logging.debug('sending stop')
        self.Command(0, 0, 0, 0, 0, 0)
        logging.debug('stop is executed')

    def forward(self, speed):
        logging.debug('sending forward with speed:{}'.format(speed))
        self.Command(speed, 0, 0, 0, 0, 0)
        logging.debug('forward is executed')

    def right(self, speed):
        logging.debug('sending right with speed:{}'.format(speed))
        self.Command(0, speed, 0, 0, 0, 0)
        logging.debug('right command is executed')

    def up(self, speed):
        logging.debug('sending up with speed:{}'.format(speed))
        self.Command(0, 0, speed, 0, 0, 0)
        logging.debug('up is executed')

    def clockwise(self, speed):
        logging.debug('sending clockwise with speed:{}'.format(speed))
        self.Command(0, 0, 0, 0, 0, speed)
        logging.debug('clockwise is executed')

    def Command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pubCommand.publish(self.command)
        self.rate.sleep()
