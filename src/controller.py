from drone import Drone
from speed_corrector import SpeedCorrector
from estimate import StateEstimate
import calendar
import time
import math
import numpy as np

EPS_LIN      = 20
EPS_ALT      = 20
EPS_ANG      = 20

PRECISION = 0.1 # used to determine sending speed to every coordinate

class Controller():
    def __init__(self):
        # Configure the four correctors for each coordinate
        self._pid_x   = SpeedCorrector()
        self._pid_y   = SpeedCorrector()
        self._pid_z   = SpeedCorrector()
        self._pid_yaw = SpeedCorrector()

        # kalman filter is used for the drone state estimation
        self._estimate = StateEstimate()

        # Used to process images and backproject them
        # this._camera  = new Camera(); TODO

        # Ensure that we don't enter the processing loop twice
        self._busy = False

        self._goal     = None

        self.inFlight = None

        # The last known state
        self._state   = {"x": 0, "y": 0, "z": 0, "yaw": 0}

        # The last time we have reached the goal (all control commands = 0)
        self._last_ok = 0;

        # Register the listener on navdata for our control loop
        def navdataListener(navdata):
                #print("seconds:", navdata.header.stamp.secs)
                self._processNavdata(navdata)
                self._control(navdata)

        # drone to manipulate
        self._drone = Drone(navdataListener)


    def isTakingOff(self):
        #print("is taking off?")
        if self.inFlight == None:
            return False

        #print(not self.inFlight)
        return not self.inFlight

    def isGoalReached(self):
        #print("is goal reached?")
        goal = self._goal

        if goal == None:
            return True
        
        #print(goal['reached'])
        return goal['reached']

    
    def isCommandExecuting(self):
        if self.isTakingOff():
            return True

        if self.inFlight:
            return not self.isGoalReached() 

        return False # we are actually landed
            

    """
        Sets the goal to the current state and attempt to hover on top.
    """
    def hover(self):
        self._go({
            "x": self._state["x"],
            "y": self._state["y"],
            "z": self._state["z"],
            "yaw": self._state["yaw"]
        });

    def land(self):
        self.inFlight = False
        self._drone.land()
        self.inFlight = None

    def takeOff(self):
        self.inFlight = False
        self._drone.takeOff()
        self.inFlight = True

    """
        Move forward (direction faced by the front camera) by the given
        distance (in meters).
    """
    def forward(self, distance):
        # Our starting position
        state = self._state

        # Remap our target position in the world coordinates
        gx = state['x'] + math.cos(state['yaw']) * distance
        gy = state['y'] + math.sin(state['yaw']) * distance

        print("forward state before goal", state)
        print("gx gy distance", gx, gy, distance)

        # Assign the new goal. 
        self._go({
            "x": gx,
            "y": gy,
            "z": state['z'],
            "yaw": state['yaw']
        })

    """
        Move backward by the given distance.
    """
    def backward(self, distance):
        print('backward distance', distance)
        self.forward(-distance)

    """
        Move right (front being the direction faced by the front camera) by the given
        distance (in meters).
    """
    def right(self, distance):
        # Our starting position
        state = self._state

        # Remap our target position in the world coordinates
        gx = state['x'] - math.sin(state['yaw']) * distance
        gy = state['y'] + math.cos(state['yaw']) * distance

        # Assign the new goal
        self._go({
            "x": gx,
            "y": gy,
            "z": state["z"],
            "yaw": state["yaw"]
        })

    """
        Move left by the given distance (in meters).
    """
    def left(self, distance):
        self.right(-distance)

    """
        Turn clockwise of the given angle. Note that this does not
        force a clockwise motion, if the angle is > 180 then the drone
        will turn in the other direction, taking the shortest path.
    """
    def clockwise(self, angle):
        state = self._state
        yaw = math.degrees(state['yaw']) + angle

        self._go({
            "x": state['x'],
            "y": state['y'],
            "z": state['z'],
            "yaw": math.radians(yaw)
        })

    """
        Turn counter clockwise of the given angle (in degrees)
    """
    def counterClockwise(self, angle):
        self.clockwise(-angle)

    """
        Climb up by the given distance (in meters).
    """
    def up(self, distance):
        state = self._state
        self._go({
            "x": state['x'],
            "y": state['y'],
            "z": state['z'] + distance,
            "yaw": state['yaw']
        })

    """
        Lower itself by the given distance (in meters).
    """
    def down(self, distance):
        self.up(-distance)

    """
        Go to the target altitude
    """
    def altitude(self, altitude):
        state = self._state
        self._go({
            "x": state['x'],
            "y": state['y'],
            "z": altitude,
            "yaw": state['yaw']
        })

    """
        Go to the target yaw (argument in degree)
    """
    def yaw(self, yaw):
        state = self._state
        self._go({
            "x": state['x'],
            "y": state['y'],
            "z": state['z'],
            "yaw": math.radians(yaw)
        })

    """
        Sets a new goal and enable the controller. When the goal
        is reached, the callback is called with the current state.

        x,y,z in meters
        yaw in degrees
    """
    def go(self, goal):
        if hasattr(goal, 'yaw'):
            goal['yaw'] = math.radians(goal['yaw'])

        return self._go(goal)

    def _go(self, goal):
        if goal == None:
            return

        # Normalize the yaw, to make sure we don't spin 360deg for nothing
        if hasattr(goal, 'yaw'):
            yaw = goal['yaw']
            goal['yaw'] = math.atan2(math.sin(yaw), math.cos(yaw))

        # Make sure we don't attempt to go too low
        #if 'z' in goal:
        #    goal['z'] = np.max(goal['z'], 1.0)

        self._goal = goal
        self._goal['reached'] = False
        #print("updated goal", self._goal)

        # Keep track of the callback to trigger when we reach the goal
        # this._callback = callback TODO  
   

    def _processNavdata(self, navdata):
        # Calculate state
        self._estimate.calculateState(navdata)

        # Keep a local copy of the state
        self._state = self._estimate.getState()

    def within(self, x, min, max):
        # there are different infinities, so it's doesn't matter
        if str(x) == float('-inf') or x < min:
            return min

        if str(x) == float('inf') or x > max:
            return max

        return x

    def _control(self, navdata):
        # Do not control if no known state or no goal defines
        if self._goal == None or self._state == None:
            return

        if self._goal['reached']:
            return

        print("goal", self._goal)
        print("state", self._state)

        # Compute error between current state and goal
        ex   = self._goal['x'] - self._state['x'] if ('x' in self._goal) else 0
        ey   = self._goal['y'] - self._state['y'] if ('y' in self._goal) else 0
        ez   = self._goal['z'] - self._state['z'] if ('z' in self._goal) else 0
        eyaw = self._goal['yaw'] - self._state['yaw'] if ('yaw' in self._goal) else 0

        # Normalize eyaw within [-180, 180]
        while eyaw < -math.pi:
            eyaw += (2 * math.pi)

        while eyaw >  math.pi:
            eyaw -= (2 * math.pi)

        # Check if we are within the target area
        if (abs(ex) < EPS_LIN) and (abs(ey) < EPS_LIN) \
            and (abs(ez) < EPS_ALT) and (abs(eyaw) < EPS_ANG):
                self._goal['reached'] = True
                print("Reached the goal!")
                return

        print("errors x y z yaw : {} {} {} {}".format(ex, ey, ez, eyaw))

        # Get Raw command from PID
        ux = self._pid_x.getAxisSpeed(ex)
        uy = self._pid_y.getAxisSpeed(ey)
        uz = self._pid_z.getAxisSpeed(ez)
        uyaw = self._pid_yaw.getAngleSpeed(eyaw)

        #print("raw speed x y z yaw : {} {} {} {}".format(ux, uy, uz, uyaw))

        # Ceil commands and map them to drone orientation
        yaw  = self._state['yaw']
        cx   = self.within(math.cos(yaw) * ux + math.sin(yaw) * uy, -1, 1)
        cy   = self.within(-math.sin(yaw) * ux + math.cos(yaw) * uy, -1, 1)
        cz   = self.within(uz, -1, 1)
        cyaw = self.within(uyaw, -1, 1)

        print("ceils speed x y z yaw : {} {} {} {}".format(cx, cy, cz, cyaw))
        #print("____")

        # Send commands to drone

        if abs(cx) > PRECISION:
            self._drone.forward(cx)
            return

        if abs(cy) > PRECISION:
            self._drone.right(cy)
            return

        if abs(cz) > PRECISION:
            self._drone.up(cz)
            return

        if abs(cyaw) > PRECISION:
            self._drone.clockwise(cyaw)
            return


# calculates elapsed seconds from 1970
def getDateNow():
    return calendar.timegm(time.gmtime())
