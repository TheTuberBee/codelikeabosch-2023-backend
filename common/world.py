from common.filter import KalmanFilter

from pydantic import BaseModel
import numpy as np

import enum
import math


class RawObject(BaseModel):
    x_rel: float
    y_rel: float
    vx_rel: float
    vy_rel: float


class Tick(BaseModel):
    host_yaw_rate: float
    host_speed: float
    objects: list[RawObject]


class ObjectType(enum.StrEnum):
    host = "host"
    env = "env"


class TrackingState(enum.StrEnum):
    candidate = "candidate"
    active = "active"
    lost = "lost"


class Measurement(BaseModel):
    x: float # m
    y: float # m
    vx: float # m/s
    vy: float # m/s

    def vectorize(self) -> np.ndarray:
        # TODO: convert input into a vector
        return KalmanFilter.const_acc_model_init_state()
    
    def measurement_matrix(self) -> np.ndarray:
        # TODO: should convert object state to measurement format
        pass
    

class HostMeasurement(BaseModel):
    vx: float # m/s
    vy: float # m/s

    def vectorize(self) -> np.ndarray:
        # TODO: convert input into a vector
        return KalmanFilter.const_acc_model_init_state()
    
    def measurement_matrix(self) -> np.ndarray:
        # TODO: should convert object state to measurement format
        pass
    

class ObjectSnapshot(BaseModel):
    x: float
    y: float
    yaw: float


class WorldSnapshot(BaseModel):
    tick: int
    time: float
    objects: list[ObjectSnapshot]


class Object:
    UPDATE_CYCLES_THRESHOLD = 5
    TIMEOUT_SECS = 3

    def __init__(
        self,
        world: "World",
        type: ObjectType,
        update_count: int = 0,
        last_update: float = None,
    ):
        self._world
        self._type = type
        self._update_count = update_count
        
        if last_update is None:
            self._last_update = world.time
        else:
            self._last_update = last_update

        self._state = KalmanFilter(
            x0 = KalmanFilter.const_acc_model_init_state(),
            P0 = KalmanFilter.const_acc_model_init_covariance(),
        )
        

    def get_tracking_state(self):
        if self._world.time - self._last_update > Object.TIMEOUT_SECS:
            return TrackingState.lost
        
        if self._update_count < Object.UPDATE_CYCLES_THRESHOLD:
            return TrackingState.candidate
        
        return TrackingState.active
    

    def predict(self, state_transition_matrix: np.ndarray):
        self._state.time_update(state_transition_matrix)


    def update(
        self, 
        measurement: Measurement, 
    ):
        z = measurement.vectorize()

        # TODO: calibrate measurement noise covariance (R)
        # TODO: modify H according to the measurement format
        self._state.measurement_update(
            z = z,
            H = np.eye(z.shape[0])
        )


    def snapshot(self):
        vx = self._state[2]
        vy = self._state[3]

        return ObjectSnapshot(
            x = self._state[0],
            y = self._state[1],
            yaw = math.atan2(vx, vy)
        )


class World:
    def __init__(self):
        self._objects: list[Object] = []
        self._tick: int = 0
        self._time: float = 0


    def tick(self, tick: Tick):
        pass


    def _snapshot(self):
        return WorldSnapshot(
            tick = self._tick,
            time = self._time,
            objects = [o.snapshot() for o in self._objects],
        )


if __name__ == "__main__":
    pass
