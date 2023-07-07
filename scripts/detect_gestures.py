#!/usr/bin/env python3

import time
from math import degrees

import rospy
import numpy as np

from tf import TransformListener, LookupException, ExtrapolationException, ConnectivityException, transformations
from std_msgs.msg import String


class GestureDetector():
    
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node("gesture_detector", anonymous=True)

        self.root_frame = "map" # NOTE : root frame, might have to change it
        update_frequency = 1.0  # NOTE : in Hz
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

    def lookup_joints(self, joints, ids=[0]) -> list:
        result = []
        for id in ids:
            for joint in joints:
                joint_name = joint + "_" + str(id)
                (tslt, rttn) = self.transform_listener.lookupTransform(
                    joint_name, self.root_frame, rospy.Time(0))
                result.append((tslt, rttn))
                print("[%s] Joint %s : \n\t trans : %s\n\t rot : %s" %
                    (self.time, joint_name, tslt, rttn))
        return result

    def _update(self) -> None:
        self.time = time.asctime(time.localtime(time.time()))
        all_joints = self.lookup_joints(self.joints)
        print(all_joints)
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

    def detect_angle(self, joints):
        transformations = self.lookup_joints(joints)
        orientations = [t[1] for t in transformations]
        angle = self.calculate_angle(*orientations)
        if angle > 85 and angle < 95:
            print("90 degrees")
        print(angle)


    def calculate_angle(self, pointA, vertex, pointB) -> float:
        r1 = transformations.quaternion_matrix(pointA)
        r2 = transformations.quaternion_matrix(vertex)
        r3 = transformations.quaternion_matrix(pointB)

        relative_rotation = r2[:3, :3].T @ r1[:3, :3]
        relative_rotation = self.pad_matrix(relative_rotation)

        inverse_relative_rotation = r2[:3, :3].T @ r3[:3, :3]
        inverse_relative_rotation = self.pad_matrix(inverse_relative_rotation)
        
        rotation_at_vertex = inverse_relative_rotation @ relative_rotation
        rotation_at_vertex = self.pad_matrix(rotation_at_vertex)

        rotation_quaternion = transformations.quaternion_from_matrix(
            rotation_at_vertex)
        angles = transformations.euler_from_quaternion(rotation_quaternion)
        angle_at_vertex = angles[2]
        angle_at_vertex_degrees = degrees(angle_at_vertex)

        return angle_at_vertex_degrees


    def pad_matrix(self, mat):
        """
        Pad a 3x3 matrix with an extra row and column to make it 4x4.
        """
        padded = np.pad(mat, ((0, 1), (0, 1)), mode='constant')
        padded[3, 3] = 1.0
        return padded


if __name__ == "__main__":
    GD = GestureDetector()
    GD.launch()