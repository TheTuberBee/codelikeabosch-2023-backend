import numpy as np

def calculate_intersection(
    car_position: np.ndarray, 
    car_angle: float,
    car_velocity: float, 
    car_yaw_rate: float, 
    pedestrian_position: np.ndarray, 
    pedestrian_angle: float, 
    pedestrian_velocity: float,
):
    epsilon: float = 1e-6  # Small value to avoid division by zero

    # Car trajectory parameters
    car_radius = car_velocity / car_yaw_rate
    car_center = car_position + car_radius * np.array([np.cos(car_angle), np.sin(car_angle)])

    # Pedestrian trajectory parameters
    pedestrian_direction = np.array([np.cos(pedestrian_angle), np.sin(pedestrian_angle)])

    # Calculate the intersection point
    relative_position = pedestrian_position - car_center
    dot_product = np.dot(relative_position, pedestrian_direction)
    intersection_point = car_center + dot_product * pedestrian_direction

    # Calculate time to intersection for the pedestrian
    pedestrian_distance_to_intersection = np.linalg.norm(intersection_point - pedestrian_position)
    pedestrian_time_to_intersection = pedestrian_distance_to_intersection / pedestrian_velocity

    # Calculate time to intersection for the car
    car_distance_to_intersection = np.linalg.norm(intersection_point - car_position)
    car_time_to_intersection = car_distance_to_intersection / car_velocity

    return intersection_point, pedestrian_time_to_intersection, car_time_to_intersection
