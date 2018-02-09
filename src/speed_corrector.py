import calendar
import time

"""
    It's used to control drone's -1..1 parameters.
    If the drone is close to goal, the parameters will reduce his speed.
"""
class SpeedCorrector():
    """
        e -- coordinate of error vector. 
    """
    def getAxisSpeed(self, e):
        if e < 4:
            return e/15
        
        return e/7

    def getAngleSpeed(self, angle):
        return angle/60
