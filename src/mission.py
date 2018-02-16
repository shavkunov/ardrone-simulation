from controller import Controller
import rospy
import time

distanceCorrection = 50

class Mission():
    def __init__(self):
        self._commands = []
        self._controller = Controller()


    def takeOff(self):
        controller = self._controller
        self._commands.append(lambda : controller.takeOff())


    def land(self):
        controller = self._controller
        self._commands.append(lambda : controller.land())


    def up(self, distance):
        controller = self._controller
        self._commands.append(lambda : controller.up(distance * distanceCorrection))


    def forward(self, distance): # in meters!
        controller = self._controller
        self._commands.append(lambda : controller.forward(distance * distanceCorrection))

    def backward(self, distance):
        controller = self._controller
        self._commands.append(lambda : controller.backward(distance * distanceCorrection))


    def clockwise(self, angle):
        controller = self._controller
        self._commands.append(lambda : controller.clockwise(angle))

    def counterClockwise(self, angle):
        controller = self._controller
        self._commands.append(lambda : controller.counterClockwise(angle))


    def hover(self):
        controller = self._controller
        self._commands.append(lambda : controller.hover())


    def execute(self):
        commands = self._commands
        controller = self._controller
        
        number = 1
        while len(commands) > 0:
            command = commands.pop(0)
       
            while controller.isCommandExecuting():
                pass
            
            print("executing mission command: ", number)
            time.sleep(1.0)
            command()

            number += 1
        
        self._commands = []
            
                
