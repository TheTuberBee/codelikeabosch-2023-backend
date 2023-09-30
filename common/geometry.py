import numpy as np

from typing import Tuple

def calculate_intersection(
    car_position: np.ndarray,           # 2 * 1, m
    car_angle: float,                   # radians
    car_velocity: float,                # m/s
    car_yaw_rate: float,                # radians/s
    pedestrian_position: np.ndarray,    # 2 * 1, m
    pedestrian_angle: float,            # radians
    pedestrian_velocity: float,         # m/s
):
    epsilon = 1e-6  # Small value to avoid division by zero

    # Car trajectory parameters
    car_radius = car_velocity / car_yaw_rate
    car_center = car_position + car_radius * np.array([np.cos(car_angle), np.sin(car_angle)])

    # Pedestrian trajectory parameters
    pedestrian_direction = np.array([np.cos(pedestrian_angle), np.sin(pedestrian_angle)])

    # Calculate the intersection point
    relative_position = pedestrian_position - car_center
    dot_product = np.sum(relative_position * pedestrian_direction)  # Dot product
    intersection_point = car_center + dot_product * pedestrian_direction
    
    # Calculate time to intersection for the pedestrian
    pedestrian_distance_to_intersection = np.linalg.norm(intersection_point - pedestrian_position)
    pedestrian_time_to_intersection = pedestrian_distance_to_intersection / pedestrian_velocity

    # Calculate time to intersection for the car
    car_distance_to_intersection = np.linalg.norm(intersection_point - car_position)
    car_time_to_intersection = car_distance_to_intersection / car_velocity

    return intersection_point, pedestrian_time_to_intersection, car_time_to_intersection[0]
