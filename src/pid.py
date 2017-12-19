import calendar
import time

"""
    Some strange calculations for me.
    It's used to control drone's -1..1 parameters.
    If the drone is close to goal, the parameters will reduce his speed.
"""
Infinity = 2e9
class PID(self, kp, ki, kd):
    def __init__(self):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self.reset()

    def reset(self):
        self._last_time = 0
        self._last_error = Infinity
        self._error_sum = 0

    def getCommand(self, e):
        # Compute dt in seconds
        time = getDateNow()
        dt = (time - self._last_time) / 1000

        de = 0
        if self._last_time != 0:
            # Compute de (error derivation)
            if self._last_error < Infinity:
                de = (e - self._last_error) / dt;

            # Integrate error
            self._error_sum += e * dt

        # Update our trackers
        self._last_time = time
        self._last_error = e

        # Compute commands
        command = self._kp * e + self._ki * self._error_sum + self._kd * de
        return command

# calculates elapsed seconds from 1970
def getDateNow():
    return calendar.timegm(time.gmtime())
