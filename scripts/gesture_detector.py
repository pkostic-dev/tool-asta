#!/usr/bin/env python3

import pickle
import time
from math import degrees, sqrt
import pprint

import rospy
import numpy as np

from tf import TransformListener, LookupException, ExtrapolationException, ConnectivityException, transformations
from std_msgs.msg import String


class GestureDetector():
    """Can detect angles between 3 joints, and set up trigger around a circle.

    The circle trigger is set up around a given joint and radius.
    """
    
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node("gesture_detector", anonymous=True)

        # Publisher of human gestures
        self.human_gesture_publisher = rospy.Publisher(
            'human_gesture',
            String,
            queue_size=1
        )

        self.root_frame = "map" # NOTE : root frame, might have to change it
        update_frequency = 10.0  # NOTE : in Hz
        self.rospy_rate = rospy.Rate(update_frequency)
        self.time = 0
        self.gesture_publisher = rospy.Publisher(
            'gesture', String, queue_size=1
            )
        self.transform_listener = TransformListener()
        self.tfs = []

        self.joints = ["head", "neck", "torso", "waist", "left_collar",
                       "left_shoulder", "left_elbow", "left_wrist",
                       "left_hand", "right_collar", "right_shoulder",
                       "right_elbow", "right_wrist", "right_hand",
                       "left_hip", "left_knee", "left_ankle",
                       "right_hip", "right_knee", "right_ankle"]
        self.treasure = self._load_joints('treasure.pkl')
        self.goal = self.treasure

        # Distance check
        duration = 2 # NOTE : shorter for debugging purpose
        self._timed_distance_checker(self.treasure, duration,
                                     self._distance_callback)

    def _timed_distance_checker(self, target_joint, duration, callback):
        self.goal = target_joint
        rospy.Timer(rospy.Duration(duration), callback)

    def _distance_callback(self, _):
        # Check distance from the treasure point
        (dist_x, dist_y, _) = self._check_distance(self.goal)
        if dist_x > 0.3:
            print("dist_x > 0.3")
        if dist_x < 0.3:
            print("dist_x < 0.3")
        if dist_y > 0.3:
            print("dist_y > 0.3")
        if dist_y < 0.3:
            print("dist_y < 0.3")
        # self.human_gesture_publisher.publish(distance)

    def _get_joint_points(self, target_joint) -> tuple:
        target_joint_name = list(target_joint.keys())[0]
        joint = self._lookup_joints([target_joint_name])

        target_x = target_joint[target_joint_name][0][0]
        target_z = target_joint[target_joint_name][0][2]
        real_x = joint[target_joint_name][0][0]
        real_z = joint[target_joint_name][0][2]

        return (target_x, target_z, real_x, real_z)

    def _check_distance(self, target_joint:dict) -> float:
        (center_x, center_z, point_x, point_z) = self._get_joint_points(
            target_joint)
        
        return calculate_distance(center_x, center_z, point_x, point_z)

    def _lookup_joints(self, joints, ids=[0]) -> dict:
        """
        Looks up the joints in the list and returns a dictionary with
        each joint as key and the translation and rotation arrays as values.
        """

        result = dict()
        for id in ids:
            for joint in joints:
                joint_name = joint + "_" + str(id)
                (tslt, rttn) = self.transform_listener.lookupTransform(
                    joint_name, self.root_frame, rospy.Time(0))
                result[joint] = (tslt, rttn)
        
        return result
    
    def _get_angle(self, joints) -> float:
        """
        Calculates the angle of the joints in list. Returns degrees.
        """

        # TODO update for dict
        transformations = self._lookup_joints(joints)
        orientations = [t[1] for t in transformations]
        angle = calculate_angle(*orientations)
        
        return angle

    def _get_joints(self, joints) -> None:
        """
        Pretty prints the joints in list.
        """

        jnts = self._lookup_joints(joints)
        pprint.pprint(jnts)

    def _circle_trigger(self, target_joint:dict, trigger:str, rad:int=0.5):
        """
        Sets up a circle trigger given a target joint around which the circle
        is set up using the radius. The joints name is used to look up the
        realtime joint. If the realtime joint is in target joint circle,
        trigger is published.
        """

        (center_x, center_z, point_x, point_z) = self._get_joint_points(
            target_joint)

        if is_point_inside_circle(center_x, center_z, rad, point_x, point_z):
            print("In circle")
            self.human_gesture_publisher.publish(trigger)

    def _load_joints(self, file_name = 'joints.pkl') -> dict:
        """
        Loads joints stored in file and returns as dictionary.
        """

        joints = dict()
        with open(file_name, 'rb') as file:
            joints = pickle.load(file)
        file.close()
        
        return joints

    def _update(self) -> None:
        """
        Update loop.
        """

        self.time = time.asctime(time.localtime(time.time()))

        # all_joints = self._lookup_joints(self.joints)
        # print(all_joints)
        # self._detect_angle(["left_wrist", "left_elbow", "left_shoulder"])
        # self._get_joints(["torso"])

        self._circle_trigger(self.treasure, "treasure_found", 0.1)

        self.rospy_rate.sleep()

    def launch(self) -> None:
        """Continuously update."""

        try:
            while not rospy.is_shutdown():
                try:
                    self._update()
                except (LookupException,
                    ConnectivityException,
                    ExtrapolationException) as exception:
                    pass
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise SystemExit


# NOTE : Helper functions.

def calculate_angle(pointA, vertex, pointB) -> float:
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


def calculate_distance(p1_x:float, p1_y:float,
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

    (_, _, distance) = calculate_distance(center_x, center_y ,point_x, point_y)

    return distance <= rad


""" TODO
2. verify if angles are correct
    -> testing
3. publish triggers for positions and angles
    -> easy
4. remake the threshold but with a rectangle not a line
    -> easy
6. other easy shapes ? (ellipse)
    -> ask chat GPT
7. threshold for 3D positions (+height) for lifting arm etc.
    -> easy ?
8. add basic gestures
    -> time consuming, need another person
    -> T pose
    -> mains en air (1 our 2, G ou D)
9. average of all points

"""

if __name__ == "__main__":
    GD = GestureDetector()
    GD.launch()