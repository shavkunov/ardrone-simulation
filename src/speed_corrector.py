import calendar
import time
import numpy as np

"""
    It's used to control drone's -1..1 parameters.
    If the drone is close to goal, the parameters will reduce his speed.
"""
class SpeedCorrector():
    """
        e -- coordinate of error vector. 
    """
    def getAxisSpeed(self, x):
        sign = 1
        if x < 0:
            sign = -1

        x = abs(x)
        return sign * (0.15*x + 1.5)/30.0

    def getAngleSpeed(self, angle):
        return angle/60.0
