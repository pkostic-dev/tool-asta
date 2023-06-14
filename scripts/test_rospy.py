#!/usr/bin/env python3

import rospy
import logging
import math
from std_msgs.msg import String
import time
import tf
import numpy as np


class SkeletonListener:
    def __init__(self):
        rospy.init_node('test_node')

        self.pub = rospy.Publisher('test_topic', String, queue_size=1)
        self.listener = tf.TransformListener()

        self.r = rospy.Rate(1.0)
        self.t = 0

        self.joints = ["head", "neck", "torso", "waist", "left_collar",
                       "left_shoulder", "left_elbow", "left_wrist", "left_hand",
                       "right_collar", "right_shoulder", "right_elbow",
                       "right_wrist", "right_hand", "left_hip", "left_knee",
                       "left_ankle", "right_hip", "right_knee", "right_ankle"]


    def lookup_joints(self, joints):
        res = []
        for joint in joints:
            lookup_joint = joint + "_0"
            (trans, rot) = self.listener.lookupTransform(
                lookup_joint, '/map', rospy.Time(0))
            res.append((trans, rot))
            print("[%s] Joint %s : \n\t trans : %s\n\t rot : %s" %
                (self.t, lookup_joint, trans, rot))
        return res


    def detect_angle(self, joints):
        transformations = self.lookup_joints(joints)
        orientations = [t[1] for t in transformations]
        angle = self.calculate_angle(*orientations)
        if angle > 85 and angle < 95:
            print("90 degrees")
        print(angle)


    def calculate_angle(self, pointA, vertex, pointB) -> float:
        r1 = tf.transformations.quaternion_matrix(pointA)
        r2 = tf.transformations.quaternion_matrix(vertex)
        r3 = tf.transformations.quaternion_matrix(pointB)

        relative_rotation = r2[:3, :3].T @ r1[:3, :3]
        relative_rotation = self.pad_matrix(relative_rotation)

        inverse_relative_rotation = r2[:3, :3].T @ r3[:3, :3]
        inverse_relative_rotation = self.pad_matrix(inverse_relative_rotation)
        
        rotation_at_vertex = inverse_relative_rotation @ relative_rotation
        rotation_at_vertex = self.pad_matrix(rotation_at_vertex)

        rotation_quaternion = tf.transformations.quaternion_from_matrix(
            rotation_at_vertex)
        angles = tf.transformations.euler_from_quaternion(rotation_quaternion)
        angle_at_vertex = angles[2]
        angle_at_vertex_degrees = math.degrees(angle_at_vertex)

        return angle_at_vertex_degrees


    def pad_matrix(self, mat):
        """
        Pad a 3x3 matrix with an extra row and column to make it 4x4.
        """
        padded = np.pad(mat, ((0, 1), (0, 1)), mode='constant')
        padded[3, 3] = 1.0
        return padded


    def run(self):
        while not rospy.is_shutdown():
            try:
                self.t = time.asctime(time.localtime(time.time()))
                #self.lookup_joints(self.joints)
                print(self.detect_angle(
                    ["left_shoulder", "left_elbow", "left_wrist"]))
            except (tf.LookupException,
                    tf.ConnectivityException,
                    tf.ExtrapolationException) as exception:
                print("[%s] Exception %s" % (self.t, exception))
            self.r.sleep()


if __name__ == "__main__":
    SL = SkeletonListener()
    SL.run()
