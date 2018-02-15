#!/usr/bin/env python

from mission import Mission
from drone import Drone
import time

def square():
    mission = Mission()

    mission.takeOff()
    mission.backward(7)
    mission.counterClockwise(90)
    mission.forward(7)
    mission.clockwise(90)
    mission.forward(7)
    mission.clockwise(90)
    mission.forward(7)
    mission.land()

    mission.execute()

def simpleTest():
    mission = Mission()

    mission.takeOff()
    
    
    mission.forward(5)
    mission.backward(5)
    mission.land()

    mission.execute()


def simpleTest2():
    mission = Mission()

    mission.takeOff()
    mission.up(15)
    mission.forward(10)
    mission.backward(10)
    mission.land()

    mission.execute()


def main():
    simpleTest()
    #square()

	
if __name__ == '__main__':
    main()
