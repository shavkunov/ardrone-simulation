import calendar
import time

"""
    Some strange calculations for me.
    It's used to control drone's -1..1 parameters.
    If the drone is close to goal, the parameters will reduce his speed.
"""
class SpeedCorrector():
    """
        e -- coordinate of error vector. 
    """
    def getAxisSpeed(self, e):
        return e/4

    def getAngleSpeed(self, angle):
        return angle/60
