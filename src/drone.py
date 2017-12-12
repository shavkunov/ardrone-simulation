from geometry_msgs.msg import Twist
from std_msgs.msg import String 
from std_msgs.msg import Empty 
from ardrone_autonomy.msg import Navdata

class Drone():
    def __init__(self):
        self.status = ""
        rospy.init_node('ardrone_flight', anonymous=False)
        self.rate = rospy.Rate(10)
        self.pubTakeoff = rospy.Publisher("ardrone/takeoff",Empty)
        self.pubLand = rospy.Publisher("ardrone/land",Empty)
        self.pubCommand = rospy.Publisher('cmd_vel',Twist)
        self.command = Twist()
        self.navdata = rospy.Subscriber("ardrone/navdata", Navdata, self.NavdataCallback)
        self.state_change_time = rospy.Time.now()    
        rospy.on_shutdown(self.Land)

    def NavdataCallback(self, navdata):
        time = navdata.header.stamp.to_sec()
        # navigation data proccessing

    def TakeOff(self):
        self.pubTakeoff.publish(Empty()) 
        self.rate.sleep()
                
    def Land(self):
        self.pubLand.publish(Empty())
    
        
    def Command(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pubCommand.publish(self.command)
        self.rate.sleep()