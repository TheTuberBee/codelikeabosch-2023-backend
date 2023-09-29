from common.filter import KalmanFilter

from pydantic import BaseModel
import numpy as np

import enum
import math
from typing import Optional


class RawObject(BaseModel):
    x_rel: float
    y_rel: float
    vx_rel: float
    vy_rel: float


class Tick(BaseModel):
    index: int
    time: float
    host_yaw_rate: float
    host_speed: float
    objects: list[Optional[RawObject]]

    def get_host_measurement(self) -> "HostMeasurement":
        return HostMeasurement(
            speed = self.host_speed,
            yaw_rate = self.host_yaw_rate,
        )

    def get_measurements(self, world: "World") -> list["Measurement"]:
        result = []
        for item in self.objects:
            if item is None:
                continue
            data = Measurement(
                x = item.x_rel + world._host.x(),
                y = item.y_rel + world._host.y(),
                vx = item.vx_rel + world._host.vx(),
                vy = item.vy_rel + world._host.vy(),
            ) 
            result.append(data)
        return result


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
        return np.array([[self.x], [self.y], [self.vx], [self.vy]])
    
    def measurement_matrix(self) -> np.ndarray:
        return np.eye(4)

    def covariance_matrix(self) -> np.ndarray:
        return np.eye(4) * 0.25 # fine tuning
    

class HostMeasurement(BaseModel):
    speed: float # m/s
    yaw_rate: float # radian/s

    def vectorize(self) -> np.ndarray:
        return np.array([[self.speed], [self.yaw_rate]])
    
    def measurement_matrix(self) -> np.ndarray:
        return np.array([
            [0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ])
    
    def covariance_matrix(self) -> np.ndarray:
        return np.eye(2) * 0.25 # fine tuning
    

class ObjectSnapshot(BaseModel):
    x: float # m
    y: float # m
    vx: float # m/s
    vy: float # m/s
    v: float # m/s
    yaw: float # radian


class WorldSnapshot(BaseModel):
    tick: int
    time: float # s
    host: ObjectSnapshot
    objects: list[ObjectSnapshot]


class Object:
    UPDATE_CYCLES_THRESHOLD = 3
    TIMEOUT_SECS = 0.4

    def __init__(
        self,
        world: "World",
        state: np.ndarray = None
    ):
        self._world = world
        self._type = type
        self._update_count = 0
        self._last_update = world.get_time()

        if state is None:
            self._state = KalmanFilter(
                x0 = KalmanFilter.const_vel_model_init_state(),
                P0 = KalmanFilter.const_vel_model_init_covariance(),
            )
        else:
            self._state = KalmanFilter(
                x0 = state,
                P0 = np.eye(state.shape[0])
            )
        

    def get_tracking_state(self):
        if self._world.get_time() - self._last_update > Object.TIMEOUT_SECS:
            return TrackingState.lost
        
        if self._update_count < Object.UPDATE_CYCLES_THRESHOLD:
            return TrackingState.candidate
        
        return TrackingState.active
    

    def time_update(self, state_transition_matrix: np.ndarray):
        self._state.time_update(state_transition_matrix)


    def measurement_update(self, measurement):
        z = measurement.vectorize()

        # TODO: calibrate measurement noise covariance (R)
        self._state.measurement_update(
            z = z,
            H = measurement.measurement_matrix(),
            R = measurement.covariance_matrix(),
        )

        self._update_count += 1
        self._last_update = self._world.get_time()


    def x(self):
        return self._state.x[0]
    
    def y(self):
        return self._state.x[1]
    

    def snapshot(self):
        return ObjectSnapshot(
            x = self.x(),
            y = self.y(),
            vx = self.vx(),
            vy = self.vy(),
            v = self.v(),
            yaw = self.yaw(),
        )
    

    def __hash__(self):
        return id(self)
    

class HostObject(Object):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
    
    def vx(self):
        return self.v() * math.cos(self.yaw())
    
    def vy(self):
        return self.v() * math.sin(self.yaw())

    def v(self):
        return self._state.x[2]

    def yaw(self):
        return self._state.x[3]
    
    def yaw_rate(self):
        return self._state.x[4]
    

class EnvObject(Object):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def vx(self):
        return self._state.x[2]
    
    def vy(self):
        return self._state.x[3]
    
    def v(self):
        return math.sqrt(self.vx() ** 2 + self.vy() ** 2)
    
    def yaw(self):
        return math.atan2(self.x(), self.y())


class World:
    ASSOCIATION_DISTANCE_THRESHOLD = 3 # m

    def __init__(self, tick: Tick):
        self._tick: int = tick.index
        self._time: float = tick.time

        vx = tick.host_speed

        self._host = HostObject(
            world = self,
            # host starts from position (0, 0)
            state = np.array([
                [0.0], # x
                [0.0], # y
                [vx],  # v
                [0.0], # yaw
                [0.0], # yaw rate
            ]),
        )

        self._objects: list[Optional[EnvObject]] = []

        measurements = tick.get_measurements(self)
        for data in measurements:
            self._add_object(data)


    def get_time(self):
        return self._time


    def tick(self, tick: Tick):

        # increment tick index
        self._tick += 1
        assert tick.index == self._tick

        # increment time
        dt = tick.time - self._time
        self._time = tick.time

        # time update

        # state transition matrix
        
        host_F = np.array([
            [1.0, 0.0, dt * math.cos(self._host.yaw()), 0.0, 0.0],
            [0.0, 1.0, dt * math.sin(self._host.yaw()), 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, dt],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ])
        self._host.time_update(host_F)

        F = KalmanFilter.const_vel_model_state_transition(dt)
        for o in self._objects:
            o.time_update(F)

        # host measurement update
        self._host.measurement_update(
            tick.get_host_measurement()
        )

        # measurement association and update
        object_set = set(self._objects)
        for data in tick.get_measurements(self):
            data: Measurement

            # based on smallest distance
            def key(item: EnvObject):
                state = item.get_tracking_state()
                if state == TrackingState.lost:
                    return math.inf
                
                distance = math.sqrt(
                    (item.x() - data.x) ** 2 + (item.y() - data.y) ** 2
                )
                item.distance = distance

                if state == TrackingState.candidate:
                    return distance * 2 # penalty
                return distance
            
            item: EnvObject = min(object_set, key = key)

            if item.distance <= World.ASSOCIATION_DISTANCE_THRESHOLD:
                item.measurement_update(data)
            else:
                self._add_object(data)

        # exporting tick data
        return self.snapshot()


    def snapshot(self):
        objects = []
        for item in self._objects:
            if item.get_tracking_state() == TrackingState.active:
                objects.append(item.snapshot())

        return WorldSnapshot(
            tick = self._tick,
            time = self._time,
            host = self._host.snapshot(),
            objects = objects,
        )
    

    def _add_object(self, measurement: Measurement):
        o = EnvObject(
            world = self,
            # for env objects the measurement and the 
            # state vectors have the same format,
            # therefore they can be used for initialization
            state = measurement.vectorize()
        )
        self._objects.append(o)



if __name__ == "__main__":
    pass
