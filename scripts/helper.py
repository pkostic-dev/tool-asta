#!/usr/bin/env python3

from math import degrees, sqrt

import numpy as np

from tf import transformations

"""
Contains shared functions that are used across other scripts, providing utility
functions and common functionalities.
"""

def calculate_degrees(pointA, vertex, pointB) -> float:
    """
    Calculates angle at vertex and returns the value in degrees.
    """

    r1 = transformations.quaternion_matrix(pointA)
    r2 = transformations.quaternion_matrix(vertex)
    r3 = transformations.quaternion_matrix(pointB)

    relative_rotation = r2[:3, :3].T @ r1[:3, :3]
    relative_rotation = pad_matrix(relative_rotation)

    inverse_relative_rotation = r2[:3, :3].T @ r3[:3, :3]
    inverse_relative_rotation = pad_matrix(inverse_relative_rotation)
    
    rotation_at_vertex = inverse_relative_rotation @ relative_rotation
    rotation_at_vertex = pad_matrix(rotation_at_vertex)

    rotation_quaternion = transformations.quaternion_from_matrix(
        rotation_at_vertex)
    angles = transformations.euler_from_quaternion(rotation_quaternion)
    angle_at_vertex = angles[2]
    angle_at_vertex_degrees = degrees(angle_at_vertex)

    return angle_at_vertex_degrees


def pad_matrix(matrix) -> np.ndarray:
    """
    Pad a 3x3 matrix with an extra row and column to make it 4x4.
    """

    padded_matrix = np.pad(matrix, ((0, 1), (0, 1)), mode='constant')
    padded_matrix[3, 3] = 1.0

    return padded_matrix


def calculate_distances(p1_x:float, p1_y:float,
                       p2_x:float, p2_y:float) -> tuple:
    """
    Calculates distance between 2 points.
    """

    distance_x:float = p2_x - p1_x
    distance_y:float = p2_y - p1_y
    distance:float = sqrt((distance_x) ** 2 + (distance_y) ** 2)
    
    return (distance_x, distance_y, distance)


def is_point_inside_circle(center_x:float, center_y:float, rad:float,
                           point_x:float, point_y:float) -> bool:
    """
    Calculates the distance between the 2 points and returns true if it's
    smaller than the radius of the circle.
    """

    (_, _, distance) = calculate_distances(center_x, center_y ,point_x, point_y)

    return distance <= rad


def translation_average(joints:dict) -> list:
    nb_joints = len(joints)
    if not nb_joints:
        return [0.0, 0.0, 0.0]

    total = [0.0, 0.0, 0.0]
    
    for joint in joints.values():
        translation = joint[0]
        total[0] += translation[0]
        total[1] += translation[1]
        total[2] += translation[2]

    average = [value / nb_joints for value in total]

    return average


def check_similar_array(array1:np.ndarray, array2:np.ndarray,
                             _decimals:int=5):
        """
        Checks if two ndarrays have similar values up to the nth digit after the decimal point.
        """

        rounded_array1 = np.round(array1, decimals=_decimals)
        rounded_array2 = np.round(array2, decimals=_decimals)

        return np.array_equal(rounded_array1, rounded_array2)


def print_red(msg:str) -> None:
    print("\033[91m{}\033[00m".format(msg))


def print_green(msg:str) -> None:
    print("\033[92m{}\033[00m".format(msg))