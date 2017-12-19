from controller import Controller
import rospy

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
        self._commands.append(lambda : controller.up(distance))


    def forward(self, distance):
        controller = self._controller
        self._commands.append(lambda : controller.forward(distance))


    def clockwise(self, angle):
        controller = self._controller
        self._commands.append(lambda : controller.clockwise(angle))


    def hover(self):
        self._controller.hover()


    def execute(self):
        commands = self._commands
        controller = self._controller
        while len(commands) > 0:
            command = commands.pop(0)

            while controller.isCommandExecuting() :
                pass # bad: active waiting
            
            command()

        self.hover()
        rospy.spin()
            
                