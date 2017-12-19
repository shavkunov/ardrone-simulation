#!/usr/bin/env python

from mission import Mission

def main():
    mission = Mission()
	mission.takeOff()
	mission.up(1.5)
	mission.forward(5)
	mission.clockwise(90)
	#mission.land()
    
    mission.execute()

	

if __name__ == '__main__':
	main()
