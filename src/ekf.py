import numpy as np

class EKF():
    def __init__(self):
         self._delta_t     = 1/15 # pure magic
         self.reset()

    def getState(self):
        return self._state

    def confidence(self):
        return self._sigma

    def reset(self):
        self._state       = {x: 0, y: 0, yaw: 0}
        self._sigma       = np.array([1, 0, 0],
                                     [0, 1, 0],
                                     [1, 0, 1])

        self._q           = np.array([0.0003, 0, 0],
                                     [0, 0.0003, 0],
                                     [0, 0, 0.0001])

        self._r           = np.array([0.3, 0, 0],
                                     [0, 0.3, 0],
                                     [0, 0, 0.3])
        self._last_yaw    = None

    def predict(data):
        # by definition of aircraft axes
        roll  = data.rotX
        pitch = data.rotY
        yaw   = data.rotZ
        vx    = data.vx / 1000 # We want m/s instead of mm/s
        vy    = data.vy / 1000
        dt    = self._delta_t

        # We are not interested by the absolute yaw, but the yaw motion,
        # so we need at least a prior value to get started.
        if this._last_yaw == null:
            this._last_yaw = yaw
            return

        # Compute the odometry by integrating the motion over delta_t
        vo = {
            dx: vx * dt,
            dy: vy * dt,
            dyaw: yaw - self._last_yaw
        }
        self._last_yaw  = yaw

        # Update the state estimate
        state = self._state
        state.x   = state.x + o.dx * np.math.cos(state.yaw) - o.dy * np.math.sin(state.yaw)
        state.y   = state.y + o.dx * np.math.sin(state.yaw) + o.dy * np.math.cos(state.yaw)
        state.yaw = state.yaw + o.dyaw;

        # Normalize the yaw value
        state.yaw = np.math.atan2(np.math.sin(state.yaw), np.math.cos(state.yaw));

        # Compute the G term (due to the Taylor approximation to linearize the function).
        G = np.array(
                [[1, 0, -1 * np.math.sin(state.yaw) * o.dx - np.math.cos(state.yaw) * o.dy],
                [0, 1,  np.math.cos(state.yaw) * o.dx - np.math.sin(state.yaw) * o.dy],
                [0, 0, 1]]
        )

        # Compute the new sigma
        self._sigma = np.sum(G.dot(self._sigma).dot(G.transpose()), (self._q))

def normAngle(rad):
    while rad >  Math.PI:
        rad -= 2 * Math.PI

    while rad < -Math.PI
        rad += 2 * Math.PI

    return rad
