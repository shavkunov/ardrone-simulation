#!/usr/bin/env python

from controller import Controller
import rospy

def main():
	controller = Controller()
	controller.takeOff()
	controller.up(1.5)
	controller.forward(5)
	controller.clockwise(90)
	controller.land()
	rospy.spin()

if __name__ == '__main__':
	main()
