import numpy as np

class EKF():
    def __init__(self):
        self._delta_t = 7.0
        self._last_yaw = None # we need to compute delta yaw
        self._state = {"x": 0, "y": 0, "yaw": 0}

    def getState(self):
        return self._state

    def predict(self, data):
        # by definition of aircraft axes
        roll  = data.rotX
        pitch = data.rotY
        yaw   = data.rotZ
        vx    = data.vx / 1000.0
        vy    = data.vy / 1000.0
        dt    = self._delta_t

        # We are not interested by the absolute yaw, but the yaw motion,
        # so we need at least a prior value to get started.
        if self._last_yaw == None:
            self._last_yaw = yaw
            return

        # Compute the odometry by integrating the motion over delta_t
        o = {
            "dx": vx * dt,
            "dy": vy * dt,
            "dyaw": yaw - self._last_yaw
        }
        self._last_yaw  = yaw

        # Update the state estimate
        state = self._state
        state['x']   = state['x'] + o["dx"] * np.math.cos(state['yaw']) - o["dy"] * np.math.sin(state['yaw'])
        state['y']   = state['y'] + o["dx"] * np.math.sin(state['yaw']) + o["dy"] * np.math.cos(state['yaw'])
        state['yaw'] = state['yaw'] + o["dyaw"]
        state['z'] = data.altd / 1000.0 # altidude in mm, we want meters
        state['vx'] = vx
        state['vy'] = vy
        print('vx', vx)


        # Normalize the yaw value
        state['yaw'] = np.math.atan2(np.math.sin(state['yaw']), np.math.cos(state['yaw']))
    
        self._state = state

def normAngle(rad):
    while rad >  Math.PI:
        rad -= 2 * Math.PI

    while rad < -Math.PI:
        rad += 2 * Math.PI

    return rad
