#!/usr/bin/env python3

import math

import rospy
import tf
import numpy as np
from PyNuitrack import py_nuitrack

from std_msgs.msg import String


class SkeletonConverter:
    """Converts Nuitrack skeleton data into TF data and publishes it.

    This class initialises a Nuitrack object. Converts the data obtained from it
    into TF skeleton data. And publishes the TF skeleton data to ROS with a
    TransformBroadcaster object. This class is a mix of an Adapter pattern class
    and	a publisher class.
    """

    def __init__(self) -> None:
        """Initialises other objects and creates class member variables

        Initialisation of class, creates Nuitrack and TransformBroadcaster
        objects. As well as all the class member variables.
        """

        super().__init__()
        rospy.init_node("tf_skeletons", anonymous=True)

        update_frequency = 1.0  # NOTE : in Hz
        self.rospy_rate = rospy.Rate(update_frequency)
        self.nuitrack = py_nuitrack.Nuitrack()
        self.transform_broadcaster = tf.TransformBroadcaster()
        # The ids of percieved skeletons
        self.ids = []
        # Presence flags
        self.present_ids = np.zeros(10, dtype=bool)
        # List containing joints of each skeleton
        # There is a total of 20 joints per skeleton
        self.joints = []
        # The translations of every joint in space
        # [ids, joints, xyz_coordinates]
        self.translation = np.zeros([10, 25, 3])
        self.sent_translation = np.zeros([10, 25, 3])
        # The rotation of every joint in space
        # [ids, joints, 3x3_matrices]
        self.rotation = np.zeros([10, 25, 9])

        self._init_nuitrack()

    def _init_nuitrack(self) -> None:
        """Initialises a Nuitrack object.

        Initialises a Nuitrack object. Searches for a Nuitrack compatible device
        and selects the first one. Checks the activation status of the license.
        Finally the Nuitrack object is run.
        """

        self.nuitrack.init()

        devices = self.nuitrack.get_device_list()
        for i, dev in enumerate(devices):
            msg = "Device :\n\tname={name}\n\tserial_number={serial_number}"
            print(
                msg.format(
                    name=dev.get_name(),
                    serial_number=dev.get_serial_number(),
                )
            )
            if i == 0:  # NOTE : always uses first detected camera
                if dev.get_activation() == "None":
                    print("No active license for device!")
                    self.nuitrack.release()
                    return

                # If a license hasn't been activated with the Nuitrack Runtime
                # package it can be activated using activate_nuitrack.py

                print(
                    "Device activation status : {activation}".format(
                        activation=dev.get_activation()
                    )
                )
                print("Setting device as active.")
                self.nuitrack.set_device(dev)

        self.nuitrack.create_modules()
        self.nuitrack.run()
        print("Nuitrack is running.")

        # Set up a shutdown hook for releasing Nuitrack from memory.
        rospy.on_shutdown(self.shutdown_hook)

    def _shutdown_hook(self) -> None:
        """Shutdown hook for the Nuitrack object."""

        if self.nuitrack != None:
            self.nuitrack.release()

    def _update(self) -> None:
        self.nuitrack.update()
        data = self.nuitrack.get_skeleton()
        self._store_skeletons(data)
        self._broadcast_skeletons()
        self.rospy_rate.sleep()

    def _store_skeletons(self, data) -> None:
        """Store the skeletons data from Nuitrack."""

        # Loop through all skeletons found
        for skeleton in range(0, len(data.skeletons)):
            # Verify if the detected skeleton's user_id has already been
            # stored
            id = data.skeletons[skeleton].user_id
            if id in self.ids:
                # If the skeleton has been detected then overwrite its
                # joints data
                id = self.ids.index(id)
                self._store_joints(id)
                self.joints[id] = data.skeletons[skeleton][1:]
            else:
                # if not, we will add now id and save data in the joints
                # list, translation list and rotation list
                self.ids.append(id)
                id = len(self.ids) - 1
                self.joints.append(data.skeletons[skeleton][1:])

    def _store_joints(self, id) -> None:
        for joint in range(0, 20):
            self.translation[id, joint, :] = (
                -np.array(self.joints[id][joint].real) / 1000.0
            ).tolist()
            self.rotation[id, joint, :] = self.joints[id][joint].orientation.flatten()

    def _broadcast_skeletons(self) -> None:
        """Function send Transform information of every point of skeletons"""

        for skeleton_id in self.ids:
            id = self.ids.index(skeleton_id)

            # Calculate the rotation in Euler angles. [joint, xyz]
            euler_rotations = np.zeros([20, 3])

            for joint_number in range(20):
                r = self.rotation[id, joint_number, :]
                m = np.mat([[r[0], r[1], r[2]], [r[3], r[4], r[5]], [r[6], r[7], r[8]]])
                euler_rotation = tf.transformations.euler_from_matrix(m, "rxyz")
                euler_rotations[joint_number, :] = euler_rotation

            # Broadcast /nuitrack_frame position and rotation
            self.broadcast_nuitrack_frame()

            joints_moving = np.ones(20, dtype=bool)
            # Send transform message for each joint
            for joint_number in range(20):
                translation = self.translation[id, joint_number, :]
                rotation = tf.transformations.quaternion_from_euler(
                    euler_rotations[joint_number, 0],
                    euler_rotations[joint_number, 1],
                    euler_rotations[joint_number, 2],
                )
                time = rospy.Time.now()
                child = str(self.joints[id][joint_number].type) + "_" + str(id)
                parent = "/nuitrack_frame"

                # Triggers based on depth and lateral position
                # z => profondeur ; x => lateral
                # if torso
                if child == "torso_%d" % id:
                    _msg = ""
                    _x = translation[0]
                    _z = translation[2]

                    # if (_z > 2):
                    # 	_msg = "human_%d_2meters" % id
                    # elif (_z > 1):
                    # 	_msg = "human_%d_1meter" % id

                    # if (_x > -0.1 and _x < 0.1):
                    # 	_msg = "human_%d_center" % id
                    # elif (_x < -0.5):
                    # 	_msg = "human_%d_left" % id
                    # elif (_x > 0.5):
                    # 	_msg = "human_%d_right" % id

                    if _x < -0:
                        _msg = "human_%d_left" % id
                    elif _x > 0:
                        _msg = "human_%d_right" % id

                    if _z < -3.0:
                        _msg += "_2m"
                    elif _z < -1.0:
                        _msg += "_1m"

                    if _msg:
                        pass
                        # self.human_gesture_publisher.publish(_msg)
                        print(_msg)
                        print("\t", _x, _z)

                if not (
                    translation == self.sent_translation[id, joint_number, :]
                ).all():
                    if self.present_ids[id] == False:
                        self.present_ids[id] = True
                    self.transform_broadcaster.sendTransform(
                        translation, rotation, time, child, parent
                    )
                else:
                    joints_moving[joint_number] = False

                self.sent_translation[id, joint_number, :] = translation

            if np.all(joints_moving == False):
                if self.present_ids[id] == True:
                    self.present_ids[id] = False
            else:
                self.human_gesture_publisher.publish(_msg)
                print("Published : " + _msg)
                print("\t", _x, _z)

    def _broadcast_nuitrack_frame(self) -> None:
        """Broadcasts the camera location tf."""

        # Translation
        x = 0
        y = 0
        z = 1
        translation = (x, y, z)  # NOTE : unit might be in meters

        # Rotation
        # Euler angles in degrees
        roll = -90.0
        pitch = 0.0
        yaw = 0.0

        # Convert degrees to radians
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        # Convert radians to a quaternion
        rotation = tf.transformations.quaternion_from_euler(
            roll_rad, pitch_rad, yaw_rad, axes="sxyz"
        )

        child = "nuitrack_frame"  # NOTE : camera location tf
        parent = "map"  # NOTE : root frame, might have to change it

        self.transform_broadcaster.sendTransform(
            translation, rotation, rospy.Time.now(), child, parent
        )

    def launch(self) -> None:
        """Continuously update Nuitrack, get skeleton data, and load it."""

        try:
            while not rospy.is_shutdown():
                self._update()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise SystemExit


if __name__ == "__main__":
    SC = SkeletonConverter()
    SC.launch()
