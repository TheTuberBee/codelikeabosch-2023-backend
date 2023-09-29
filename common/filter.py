import numpy as np

class KalmanFilter:
    def __init__(
        self, 
        x0: np.ndarray, 
        P0: np.ndarray, 
    ):
        # initial state vector - n(x) * 1
        self.x = x0.copy() 
        # initial covariance matrix - n(x) * n(x)
        self.P = P0.copy()


    def time_update(
        self,
        F: np.ndarray,  # state transition matrix - n(x) * n(x)
    ):
        """
        F: state transition matrix 
            - translates state vector to an estimated state vector
        """

        # state extrapolation
        self.x = F @ self.x

        # covariance extrapolation
        self.P = F @ self.P @ F.T


    def measurement_update(
        self,
        z: np.ndarray,          # measurement vector - n(z) * 1
        H: np.ndarray,   # measurement matrix - n(z) * n(x)
        R: np.ndarray = None,   # measurement noise covariance matrix - n(z) * n(z)
    ):
        """
        z: measurement vector
        H: measurement matrix 
            - translates state vector to a measurement vector
        R: measurement noise covariance matrix
            - high values mean confidence in prediction (infinity: measurement = prediction)
            - low values mean confidence in measurement (0: measurement = new state)
        """

        if H is None:
            H = np.eye(self.x.shape[0])

        if R is None:
            R = np.zeros((z.shape[0], z.shape[0]))

        # kalman gain - n(x) * n(z)
        K = self.P @ H.T @ np.linalg.inv(H @ self.P @ H.T + R)

        print("kalman gain: \n", K)

        # state update
        self.x = self.x + K @ (z - H @ self.x)

        # covariance update
        self.P = (np.eye(self.P.shape[0]) - K @ H) @ self.P


    @classmethod
    def const_acc_model_state_transition(
        cls,
        dt: float,  # time step
    ):
        """for objects on 2d plane"""
        return np.array([
            [1.0, 0.0, dt, 0.0, 0.5 * dt**2, 0.0],
            [0.0, 1.0, 0.0, dt, 0.0, 0.5 * dt**2],
            [0.0, 0.0, 1.0, 0.0, dt, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ])
    

    @classmethod
    def const_acc_model_init_state(cls):
        """for objects on 2d plane"""
        return np.array([
            [0.0], # x(m)
            [0.0], # y(m)
            [0.0], # vx(m/s)
            [0.0], # vy(m/s)
            [0.0], # ax(m/s^2)
            [0.0], # ay(m/s^2)
        ])
    

    @classmethod
    def const_acc_model_init_covariance(cls):
        """for objects on 2d plane"""
        return np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        ])


if __name__ == "__main__":
    import time

    hidden_state = KalmanFilter.const_acc_model_init_state()
    hidden_state[2] = 1

    x0 = KalmanFilter.const_acc_model_init_state()
    P0 = KalmanFilter.const_acc_model_init_covariance()

    filter = KalmanFilter(x0, P0)

    while True:
        time.sleep(1)

        hidden_state[0] += 1

        print(f"hidden state: \n{hidden_state}")

        F = KalmanFilter.const_acc_model_state_transition(dt = 1)

        print(f"state transition: \n{F}")

        filter.time_update(F)

        print(f"predicted state: \n{filter.x}")

        z = hidden_state + np.random.normal(0, 1, hidden_state.shape)

        print(f"measurement: \n{z}")

        H = np.eye(hidden_state.shape[0])
        R = np.eye(hidden_state.shape[0])

        filter.measurement_update(z, H, R)

        print(f"new state: \n{filter.x}")
        print(f"mew covariance: \n{filter.P}")
