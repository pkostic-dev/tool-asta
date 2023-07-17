#!/usr/bin/env python3

import time
from math import degrees, sqrt
import pprint

import rospy
import numpy as np

from tf import TransformListener, LookupException, ExtrapolationException, ConnectivityException, transformations
from std_msgs.msg import String


class GestureDetector():
    
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node("gesture_detector", anonymous=True)

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

    def _lookup_joints(self, joints, ids=[0]) -> dict:
        result = dict()
        for id in ids:
            for joint in joints:
                joint_name = joint + "_" + str(id)
                (tslt, rttn) = self.transform_listener.lookupTransform(
                    joint_name, self.root_frame, rospy.Time(0))
                result[joint] = (tslt, rttn)
        return result
    
    def _detect_angle(self, joints):
        # TODO update for dict
        transformations = self._lookup_joints(joints)
        orientations = [t[1] for t in transformations]
        angle = calculate_angle(*orientations)
        print(angle)

    def _detect_joints(self, joints) -> dict:
        jnts = self._lookup_joints(joints)
        pprint.pprint(jnts)

    def _update(self) -> None:
        self.time = time.asctime(time.localtime(time.time()))
        # all_joints = self._lookup_joints(self.joints)
        # print(all_joints)
        #self._detect_angle(["left_wrist", "left_elbow", "left_shoulder"])
        self._detect_joints(["torso"])
        self.rospy_rate.sleep()

    def launch(self) -> None:
        """Continuously listen for tf data, and publish gestures."""

        try:
            while not rospy.is_shutdown():
                try:
                    self._update()
                except (LookupException,
                    ConnectivityException,
                    ExtrapolationException) as exception:
                    pass
                    # msg = "[{t}] Exception {exception}"
                    # print(msg.format(t=self.time, exception=exception))
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise SystemExit
        
    ###############




def calculate_angle(pointA, vertex, pointB) -> float:
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
    print(type(padded_matrix))
    return padded_matrix


def is_point_inside_circle(center_x, center_y, rad, point_x, point_y) -> bool:
    distance = sqrt((point_x - center_x) ** 2 + (point_y - center_y) ** 2)
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