from common.filter import KalmanFilter

from pydantic import BaseModel
import numpy as np

import enum
from typing import Optional


class ObjectType(enum.StrEnum):
    unknown = "unknown"
    host = "host"
    car = "car"
    pedestrian = "pedestrian"


class TrackingState(enum.StrEnum):
    candidate = "candidate"
    active = "active"
    lost = "lost"


class Measurement(BaseModel):
    # TODO: figure out what input fields do we have

    def vectorize(self) -> np.ndarray:
        # TODO: convert input into a vector
        return KalmanFilter.const_acc_model_init_state()


class Object:
    UPDATE_CYCLES_THRESHOLD = 5
    TIMEOUT_SECS = 3

    def __init__(
        self,
        world: "World",
        type: ObjectType = ObjectType.unknown,
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
        self._state.measurement_update(
            z = z,
            H = np.eye(z.shape[0])
        )



class World:
    def __init__(self):
        self._objects: list[Object] = []
        self._time: float = 0

    def update(self, dt: float):
        self._time += dt


if __name__ == "__main__":
    pass
