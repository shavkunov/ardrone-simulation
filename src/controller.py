from drone import Drone
from pid import PID
from pid import getDateNow
from ekf import EKF
import calendar
import time
import math

EPS_LIN      = 0.1; # We are ok with 10 cm horizontal precision
EPS_ALT      = 0.1; # We are ok with 10 cm altitude precision
EPS_ANG      = 0.1; # We are ok with 0.1 rad precision (5 deg)
STABLE_DELAY = 5; # Time in s to wait before declaring the drone on target

class Controller():
    def __init__(self):

        #The position of a roundel tag to detect
        self._tag = {"x": 0, "y": 0, "yaw": 0} # TODO: what?

        # Configure the four PID required to control the drone
        self._pid_x   = PID(0.5, 0, 0.35)
        self._pid_y   = PID(0.5, 0, 0.35)
        self._pid_z   = PID(0.8, 0, 0.35)
        self._pid_yaw = PID(1.0, 0, 0.30)

        # kalman filter is used for the drone state estimation
        self._ekf = EKF()

        # Used to process images and backproject them
        # this._camera  = new Camera(); TODO

        # Control will only work if enabled
        self._enabled = False

        # Ensure that we don't enter the processing loop twice
        self._busy = False

        # The current target goal and an optional callback to trigger when goal is reached
        self._goal     = None
        self._callback = None

        # The last known state
        self._state   = {"x": 0, "y": 0, "z": 0, "yaw": 0}

        # The last time we have reached the goal (all control commands = 0)
        self._last_ok = 0;

        # Register the listener on navdata for our control loop
        def navdataListener(navdata):
                self._busy = True
                self._processNavdata(navdata)
                self._control(navdata)
                self._busy = False

        # drone to manipulate
        self._drone = Drone(navdataListener)

    """
        Enable auto-pilot. The controller will attempt to bring
        the drone (and maintain it) to the goal.
    """
    def enable(self):
        self._pid_x.reset()
        self._pid_y.reset()
        self._pid_z.reset()
        self._pid_yaw.reset()
        self._enabled = True

    """
        Disable auto-pilot. The controller will stop all actions
        and send a stop command to the drone.
    """
    def disable(self):
        self._enabled = False
        self._drone.stop()

    """
        True if drone is busy and executing command.
    """
    def isBusy(self):
        goal_flag = True

        if self._goal != None:
            goal_flag = self._goal["reached"]

        return self._busy or not goal_flag

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
        self._busy = True
        self._drone.land()
        self._busy = False

    def takeOff(self):
        self._busy = True
        self._drone.takeOff()
        self._busy = False

    """
        Reset the kalman filter to its base state (default is x:0, y:0, yaw:0).
        This is especially usefull to set mark the drone position as the starting position
        after takeoff. We must disable, to ensure that the zeroing does not trigger a sudden move
        of the drone.
    """
    def zero(self):
        self.disable()
        self._ekf.reset()

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

        # Assign the new goal. TODO: callback?
        self._go({
            "x": gx,
            "y": gy,
            "z": state['z'],
            "yaw": state['yaw']
        })

    """
        Move backward by the given distance (in meters).
    """
    def backward(self, distance):
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
    # TODO: add callback
    def go(self, goal):
        if hasattr(goal, 'yaw'):
            goal['yaw'] = math.radians(goal['yaw'])

        return self._go(goal)

    def _go(self, goal=None):
        # Since we are going to modify goal settings, we disable the controller, just in case.
        self.disable()

        # If no goal given, assume an empty goal
        if goal == None:
            goal = {}

        # Normalize the yaw, to make sure we don't spin 360deg for nothing
        if hasattr(goal, 'yaw'):
            yaw = goal['yaw']
            goal['yaw'] = math.atan2(math.sin(yaw), math.cos(yaw))

        # Make sure we don't attempt to go too low
        if hasattr(goal, 'z'):
            goal['z'] = math.max(goal['z'], 0.5)

        # Update our goal
        self._goal = goal
        self._goal['reached'] = False

        #print("updated goal", self._goal)
        #print("state", self._state)

        # Keep track of the callback to trigger when we reach the goal
        # this._callback = callback TODO

        # (Re)-Enable the controller
        self.enable()

    def _processNavdata(self, navdata):
        # EKF prediction step
        self._ekf.predict(navdata)

        # Keep a local copy of the state
        self._state = self._ekf.getState()
        self._state['z'] = navdata.altd / 1000 # altidude in mm, we want meters
        self._state['vx'] = navdata.vx / 1000 # We want m/s instead of mm/s
        self._state['vy'] = navdata.vy / 1000

    def within(self, x, min, max):
        if x < min:
            return min

        if x > max:
            return max

        return x

    def _control(self, navdata):
        #print("control_____")
        #print("updated goal", self._goal)
        #print("state", self._state)

        # Do not control if not enabled
        if not self._enabled:
            return

        # Do not control if no known state or no goal defines
        if self._goal == None or self._state == None:
            return

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
            #print("within targer area")
            # Have we been here before ?
            if not self._goal['reached'] and self._last_ok != 0:
                # And for long enough ?
                #print("checking for stable delay")
                #print("stable diff", getDateNow() - self._last_ok)
                if (getDateNow() - self._last_ok) > STABLE_DELAY:
                    # Mark the goal has reached
                    self._goal['reached'] = True

                    # We schedule the callback in the near future. This is to make
                    # sure we finish all our work before the callback is called.
                    if self._callback != None:
                        # TODO: sleep for 10 ms
                        # setTimeout(self._callback, 10)
                        self._callback = None

                    # Emit a state reached
                    # self.emit('goalReached', this._state);
            else:
                self._last_ok = getDateNow()
        else:
            # If we just left the goal, we notify
            if self._last_ok != 0:
                # Reset last ok since we are in motion
                self._last_ok = 0
                self._goal['reached'] = False
                # self.emit('goalLeft', this._state);

        # Get Raw command from PID
        ux = self._pid_x.getCommand(ex)
        uy = self._pid_y.getCommand(ey)
        uz = self._pid_z.getCommand(ez)
        uyaw = self._pid_yaw.getCommand(eyaw)

        # Ceil commands and map them to drone orientation
        yaw  = self._state['yaw']
        cx   = self.within(math.cos(yaw) * ux + math.sin(yaw) * uy, -1, 1)
        cy   = self.within(-math.sin(yaw) * ux + math.cos(yaw) * uy, -1, 1)
        cz   = self.within(uz, -1, 1)
        cyaw = self.within(uyaw, -1, 1)

        # Emit the control data for auditing
        # TODO : add this as property for debugging
        """this.emit('controlData', {
            state:   this._state,
            goal:    this._goal,
            error:   {ex: ex, ey: ey, ez: ez, eyaw: eyaw},
            control: {ux: ux, uy: uy, uz: uz, uyaw: uyaw},
            last_ok: this._last_ok,
            tag:     (d.visionDetect && d.visionDetect.nbDetected > 0) ? 1 : 0
        });"""

        # Send commands to drone
        if abs(cx) > 0.01:
            self._drone.forward(cx)

        if abs(cy) > 0.01:
            self._drone.right(cy)

        if abs(cz) > 0.01:
            self._drone.up(cz)

        if abs(cyaw) > 0.01:
            self._drone.clockwise(cyaw)
