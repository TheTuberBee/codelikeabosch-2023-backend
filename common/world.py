from common.filter import KalmanFilter

from pydantic import BaseModel
import numpy as np

import enum
import math
from typing import Optional


# normalised but relative data from the dataset
class RawObject(BaseModel):
    x_rel: float # m
    y_rel: float # m
    vx_rel: float # m/s
    vy_rel: float # m/s


# one row of the dataset
class Tick(BaseModel):
    index: int
    time: float
    host_yaw_rate: float
    host_speed: float
    objects: list[Optional[RawObject]]

    # transforms tick data to a measurement for the host object
    def get_host_measurement(self) -> "HostMeasurement":
        return HostMeasurement(
            speed = self.host_speed,
            yaw_rate = self.host_yaw_rate,
        )

    # transforms tick data to a list of measurements for the environmental objects
    def get_measurements(self, world: "World") -> list["Measurement"]:
        result = []
        for item in self.objects:
            if item is None:
                continue
            yaw = world._host.yaw()
            # transforming relative coordinates to world coordinates
            data = Measurement(
                x = item.x_rel * math.cos(yaw) - item.y_rel * math.sin(yaw) + world._host.x(),
                y = item.x_rel * math.sin(yaw) + item.y_rel * math.cos(yaw) + world._host.y(),
                vx = item.vx_rel * math.cos(yaw) - item.vy_rel * math.sin(yaw) + world._host.vx(),
                vy = item.vx_rel * math.sin(yaw) + item.vy_rel * math.cos(yaw) + world._host.vy(),
            )
            result.append(data)
        return result


# measurement for an enviromental object
class Measurement(BaseModel):
    x: float # m
    y: float # m
    vx: float # m/s
    vy: float # m/s

    # Kalman filter state vector
    def vectorize(self) -> np.ndarray:
        return np.array([[self.x], [self.y], [self.vx], [self.vy]])
    
    # Kalman filter measurement matrix
    def measurement_matrix(self) -> np.ndarray:
        return np.eye(4)

    # Kalman filter measurement covariance matrix
    def covariance_matrix(self) -> np.ndarray:
        return np.eye(4) * 0.25 # fine tuning
    

# measurement for the host object
class HostMeasurement(BaseModel):
    speed: float # m/s
    yaw_rate: float # radian/s

    # Kalman filter state vector
    def vectorize(self) -> np.ndarray:
        return np.array([[self.speed], [self.yaw_rate]])
    
    # Kalman filter measurement matrix
    def measurement_matrix(self) -> np.ndarray:
        return np.array([
            [0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ])
    
    # Kalman filter measurement covariance matrix
    def covariance_matrix(self) -> np.ndarray:
        return np.eye(2) * 0.25 # fine tuning
    

# constant attributes of an object
class ObjectMeta(BaseModel):
    color: str

# time dependent attributes of an object
class ObjectSnapshot(BaseModel):
    x: float # m
    y: float # m
    vx: float # m/s
    vy: float # m/s
    v: float # m/s
    yaw: float # radian

# time dependent attributes of the world,
# objects are identified by string ids
class WorldSnapshot(BaseModel):
    tick: int
    time: float # s
    host: ObjectSnapshot
    objects: dict[str, ObjectSnapshot]
    events: list[str]

# output of the whole process
class Output(BaseModel):
    object_meta: dict[str, ObjectMeta]
    snapshots: list[WorldSnapshot]


# stages of the lifecycle of an object
class TrackingState(enum.StrEnum):
    # not yet shown, can recieve updates
    candidate = "candidate"
    # shown on frontend, prioritized for updates
    active = "active" 
    # not shown, cannot recieve updates anymore
    lost = "lost" 


# base class for the enviromental objects 
# and the host object ("ego vehicle"),
# encapsulates a Kalman filter
class Object:
    # number of updates to wait before an object 
    # is considered actively tracked
    UPDATE_TICKS_THRESHOLD = 3
    # number of seconds to wait before an object
    # is considered lost
    TIMEOUT_SECS = 1.0

    # shown on the frontend
    DEFAULT_COLOR = "#AAAA00"
    COLLIDER_COLOR = "#CC00CC"

    def __init__(
        self,
        world: "World",
        state: np.ndarray = None
    ):
        # reference to owner world
        self._world = world

        # color of the object on the frontend
        self._color = Object.DEFAULT_COLOR

        # lifecycle attributes
        self._update_count = 0
        self._last_update = world.get_time() # s

        # unique id
        self._id: str = world.next_id()

        # first appearance
        self._first_tick: int = world._tick

        # Kalman filter state initialization
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


    def x(self):
        return self._state.x[0]
    
    def y(self):
        return self._state.x[1]
    
    def get_id(self):
        return self._id
    
    def get_first_tick(self):
        return self._first_tick
    
    def mark_as_collider(self):
        self._color = Object.COLLIDER_COLOR
        

    def get_tracking_state(self):
        if self._world.get_time() - self._last_update > Object.TIMEOUT_SECS:
            return TrackingState.lost
        
        if self._update_count < Object.UPDATE_TICKS_THRESHOLD:
            return TrackingState.candidate
        
        return TrackingState.active
    

    # Kalman filter state extrapolation
    def time_update(self, state_transition_matrix: np.ndarray):
        # process noise covariance
        Q = np.eye(state_transition_matrix.shape[0]) * 0.25 # fine tuning
        self._state.time_update(state_transition_matrix, Q)


    # Kalman filter state correction
    def measurement_update(self, measurement):
        z = measurement.vectorize()

        self._state.measurement_update(
            z = z,
            H = measurement.measurement_matrix(),
            R = measurement.covariance_matrix(),
        )

        self._update_count += 1
        self._last_update = self._world.get_time()
    

    def get_object_meta(self):
        return ObjectMeta(
            color = self._color
        )


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
    ASSOCIATION_DISTANCE_THRESHOLD = 2.5 # m
    COLLISION_THRESHOLD = 1.2 # m from midpoint of frontal structure
    CAMERA_FRONT_DISTANCE = 3.2 # m
    COLLISION_BACKTRACK_TICKS = 20

    def __init__(self, tick: Tick):
        self._snapshots: list[WorldSnapshot] = []
        self._tick: int = tick.index
        self._time: float = tick.time
        self._last_id = -1

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
    

    def get_snapshots(self):
        return self._snapshots


    def tick(self, tick: Tick):
        events: list[str] = []

        # incrementing tick index
        self._tick += 1
        assert tick.index == self._tick

        # time update (state extrapolation)
        self._time_update(tick.time)

        # measurement update (state correction)
        self._measurement_update(tick)

        # exporting tick data
        snapshot = self.snapshot(events)
        self._snapshots.append(snapshot)

        self._detect_collisions(events)



        if len(events) > 0:
            print(self._tick, events)


    def get_object_meta(self):
        result = {}
        for item in self._objects:
            result[str(item.get_id())] = item.get_object_meta()
        return result


    def snapshot(self, events: list[str] = []):
        objects = {}
        for item in self._objects:
            if item.get_tracking_state() == TrackingState.active:
                objects[str(item.get_id())] = item.snapshot()
        
        return WorldSnapshot(
            tick = self._tick,
            time = self._time,
            host = self._host.snapshot(),
            objects = objects,
            events = events,
        )
    

    def _time_update(self, time):
        # increment time
        dt = time - self._time
        self._time = time

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


    def _measurement_update(self, tick: Tick):
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
    

    def _add_object(self, measurement: Measurement):
        o = EnvObject(
            world = self,
            # for env objects the measurement and the 
            # state vectors have the same format,
            # therefore they can be used for initialization
            state = measurement.vectorize()
        )
        self._objects.append(o)


    def next_id(self):
        self._last_id += 1
        return str(self._last_id)
    

    def _detect_collisions(self, events: list[str]):
        # estimating position of 50% width at frontal structure of host
        front_pos = np.array([
            [self._host.x() + World.CAMERA_FRONT_DISTANCE * math.cos(self._host.yaw())],
            [self._host.y() + World.CAMERA_FRONT_DISTANCE * math.sin(self._host.yaw())],
        ])

        for obj in self._objects:
            if obj.get_tracking_state() != TrackingState.active:
                continue

            obj_pos = np.array([
                [obj.x()], 
                [obj.y()],
            ])

            # calculating distance between host and object
            distance = np.linalg.norm(front_pos - obj_pos)

            if distance <= World.COLLISION_THRESHOLD and obj._color != Object.COLLIDER_COLOR:
                obj.mark_as_collider()
                self._classify_collision(obj, events)


    def _classify_collision(self, obj: EnvObject, events: list[str]):
        # backtracking host and object displacement for max 10 ticks
        first_tick_index = max(
            obj.get_first_tick(), 
            self._tick - World.COLLISION_BACKTRACK_TICKS
        )

        first_tick = self._snapshots[first_tick_index]
        last_tick = self._snapshots[-1]

        r_host = np.array([
            [first_tick.host.x - last_tick.host.x],
            [first_tick.host.y - last_tick.host.y],
        ])

        obj_first = first_tick.objects[str(obj.get_id())]
        obj_last = last_tick.objects[str(obj.get_id())]

        r_obj = np.array([
            [obj_first.x - obj_last.x],
            [obj_first.y - obj_last.y],
        ])

        # calculating angle between displacements
        alpha = math.acos(
            np.dot(r_host.T, r_obj) / (np.linalg.norm(r_host) * np.linalg.norm(r_obj))
        )

        d_yaw = abs(first_tick.host.yaw - last_tick.host.yaw)
        dt = last_tick.time - first_tick.time
        avg_yaw_rate = d_yaw / dt

        # we assume the host is turning if it has a yaw rate above 10°/s
        if avg_yaw_rate > 0.17:
            # CPTA = car to pedestrian turn adult
            events.append(f"CPTA with object #{obj.get_id()}")

        # alpha < 30° means the host and the object is moving
        # roughly in the same direction
        elif alpha < math.pi / 6:
            # CPLA = car to pedestrian longitudinal adult
            events.append(f"CPLA with object #{obj.get_id()}")

        # in every other case we fall back to CPNCO
        else:
            # CPNCO = car to pedestrian nearside child obstructed
            events.append(f"CPNCO with object #{obj.get_id()}")


if __name__ == "__main__":
    pass
