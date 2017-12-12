#!/usr/bin/env python 

#import library ros 
import rospy 
import time

#import library untuk mengirim command dan menerima data navigasi dari quadcopter
from geometry_msgs.msg import Twist
from std_msgs.msg import String 
from std_msgs.msg import Empty 
from ardrone_autonomy.msg import Navdata

COMMAND_PERIOD = 1000


class AutonomousFlight():
    def __init__(self):
        self.status = ""
        rospy.init_node('forward', anonymous=False)
        self.rate = rospy.Rate(10) # float(1.0/3.0)
        self.pubTakeoff = rospy.Publisher("ardrone/takeoff",Empty)
        self.pubLand = rospy.Publisher("ardrone/land",Empty)
        self.pubCommand = rospy.Publisher('cmd_vel',Twist)
        self.command = Twist()
        self.navdata = rospy.Subscriber("ardrone/navdata", Navdata, self.NavdataCallback)
        #self.commandTimer = rospy.Timer(rospy.Duration(COMMAND_PERIOD/1000.0),self.SendCommand)
        self.state_change_time = rospy.Time.now()    
        rospy.on_shutdown(self.SendLand)

    def NavdataCallback(self, navdata):
        t = navdata.header.stamp.to_sec()
        #print("received odometry message: time=%f battery=%f vx=%f vy=%f z=%f yaw=%f"%(t,navdata.batteryPercent,navdata.vx,navdata.vy,navdata.altd,navdata.rotZ))
        print(navdata)

    def SendTakeOff(self):
        self.pubTakeoff.publish(Empty()) 
        self.rate.sleep()
                
    def SendLand(self):
        self.pubLand.publish(Empty())
    
        
    def SetCommand(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        self.command.linear.x = linear_x
        self.command.linear.y = linear_y
        self.command.linear.z = linear_z
        self.command.angular.x = angular_x
        self.command.angular.y = angular_y
        self.command.angular.z = angular_z
        self.pubCommand.publish(self.command)
        self.rate.sleep()

if __name__ == '__main__': 
    try: 
        i = 0
        uav = AutonomousFlight()
         
        time.sleep(3)
        uav.SendTakeOff()
        
        time.sleep(3)
        uav.SetCommand(0,0,1,0,0,0)

        time.sleep(3)
        uav.SetCommand(1,0,0,0,0,0)

        # clockwise turn
        time.sleep(3)
        uav.SetCommand(0,0,0,0,0,-1)

        time.sleep(3)
        uav.SetCommand(0,0,0,0,0,-1)

        time.sleep(3)
        uav.SetCommand(0,0,0,0,0,-1)

        time.sleep(3)
        uav.SetCommand(0,0,0,0,0,-1)

        time.sleep(3)
        uav.SendLand()

        
		
        # uav.SendTakeOff()
        """while not rospy.is_shutdown():
            if i <= 1:
                uav.SendTakeOff()
            i += 1
            if i <= 30 :
                uav.SetCommand(0,0,1,0,0,0)
                i+=1
            elif i<=60 :
                uav.SetCommand(1,0,0,0,0,0)
                i+=1
            else:
                uav.SetCommand(0,0,0,0,0,0)"""
         
    except rospy.ROSInterruptException:
        pass
