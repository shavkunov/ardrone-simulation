#!/usr/bin/env python

from mission import Mission
from drone import Drone
import time

def main():
    mission = Mission()
    mission.takeOff()
    #mission.forward(5)
    #mission.land()

    mission.execute()

	
if __name__ == '__main__':
    main()
