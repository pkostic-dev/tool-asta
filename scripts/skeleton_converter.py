#!/usr/bin/env python3

from math import radians

import rospy
import numpy as np

from tf import TransformBroadcaster, transformations
from PyNuitrack import py_nuitrack # type: ignore
from helper import check_similar_array

# Robotact spectacle
from std_msgs.msg import String

class SkeletonConverter():
    """Converts Nuitrack skeleton data into TF data and publishes it.

    This class initialises a Nuitrack object. Converts the data obtained from
    it into TF skeleton data. And publishes the TF skeleton data to ROS with a
    TransformBroadcaster object. This class is a mix of an Adapter pattern
    class and a publisher class. It doesn't publish joints with less than 0.5
    confidence and it doesn't publish the same joint data twice.
    """

    def __init__(self) -> None:
        """Initialises other objects and creates class member variables

        Initialisation of class, creates Nuitrack and TransformBroadcaster
        objects. As well as all the class member variables.
        """

        super().__init__()
        rospy.init_node("tf_skeletons", anonymous=True)
        self.nb_joints = 20
        self.max_ids = 10
        self.camera_frame = "nuitrack_frame" # NOTE : camera location tf
        self.root_frame = "map" # NOTE : root frame, might have to change it
        self.translation_scale = 1000.0
        update_frequency = 10.0  # NOTE : in Hz
        self.rospy_rate = rospy.Rate(update_frequency)
        self.nuitrack = py_nuitrack.Nuitrack()
        self.transform_broadcaster = TransformBroadcaster()
        # The ids of percieved skeletons
        self.ids = []
        # List containing joints of each skeleton (20 joints per skeleton)
        self.joints = []
        # The translations of every joint in space [ids, joints, xyz]
        self.translation = np.zeros([self.max_ids, self.nb_joints, 3])
        # The rotation of every joint in space [ids, joints, matrix]
        self.rotation = np.zeros([self.max_ids, self.nb_joints, 9])
        # The confidence of every joint (0.0 to 1.0 theoretically)
        self.confidence = np.zeros([self.max_ids, self.nb_joints])
        # The previously sent translations of every joint in space
        self.last_translation = np.zeros([self.max_ids, self.nb_joints, 3])

        # Robotact Spectacle
        self.human_position_publisher = rospy.Publisher(
            'human_position',
            String,
            queue_size=1
        )

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
            if i == 0: # NOTE : always uses first detected camera
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
        rospy.on_shutdown(self._shutdown_hook)

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
        for skeleton in range(len(data.skeletons)):
            # Verify if the detected skeleton's user_id has already been
            # stored
            id = data.skeletons[skeleton].user_id
            joints = data.skeletons[skeleton][1:]
            if id in self.ids:
                # If the skeleton has been detected then overwrite its
                # joints data
                id_index = self.ids.index(id)
                self.joints[id_index] = joints
            else:
                # if not, we will add now id and save data in the joints
                # list, translation list and rotation list
                self.ids.append(id)
                id_index = len(self.ids) - 1
                self.joints.append(joints)
            self._store_joint_values(id_index)

    def _store_joint_values(self, id) -> None:
        """Store joint values into separate lists"""

        for joint in range(self.nb_joints):
            joint_translation = self.joints[id][joint].real
            scaled = -np.array(joint_translation) / self.translation_scale
            self.translation[id, joint, :] = (scaled).tolist()
            joint_rotation = self.joints[id][joint].orientation
            flattened = joint_rotation.flatten()
            self.rotation[id, joint, :] = flattened
            joint_confidence = self.joints[id][joint].confidence
            self.confidence[id, joint] = joint_confidence

    def _broadcast_skeletons(self) -> None:
        """Function send Transform information of every point of skeletons"""

        for id in self.ids:
            id_index = self.ids.index(id)

            # Broadcast /nuitrack_frame position and rotation (camera)
            self._broadcast_nuitrack_frame()

            # Broadcast transform message for each joint
            msg = str(rospy.Time.now()) + " "
            for joint in range(self.nb_joints):
                # Rotation to euler
                rotations = self.rotation[id_index, joint, :]
                matrix = np.mat(
                    [
                        [rotations[0], rotations[1], rotations[2]],
                        [rotations[3], rotations[4], rotations[5]],
                        [rotations[6], rotations[7], rotations[8]],
                    ]
                )
                euler_rotation = transformations.euler_from_matrix(
                    matrix, "rxyz")

                last_translation = self.last_translation[id_index, joint, :]
                translation = self.translation[id_index, joint, :]
                rotation = transformations.quaternion_from_euler(
                    *euler_rotation
                )
                time = rospy.Time.now()
                joint_name = str(self.joints[id_index][joint].type)
                child = joint_name + "_" + str(id_index)
                parent = "/nuitrack_frame"

                # Currently confidence is either 0.0 or 0.75
                
                confidence = self.confidence[id_index][joint]
                similar = check_similar_array(
                    translation,
                    last_translation)
                
                # NOTE : Confidence is not enough to get rid of invalid
                #        points (static invisible people), so we check
                #        for very similar translations and don't publish
                #        them.
                if ((confidence > 0.5) and not similar):
                    # msg = "[{time}] {child}({conf}) = {tsl} {rot}"

                    # print(msg.format(time=time, child=child, conf=confidence,
                    #     tsl=translation, rot=rotation))
                    msg += child + ", "
                    self.transform_broadcaster.sendTransform(
                        translation, rotation, time, child, parent
                    )
                    self.last_translation[id_index, joint, :] = translation

                    # Robotact Spectacle
                    self._publish_human_position(joint_name, translation)
            #print(msg)

    def _publish_human_position(self, joint_name, translation):
        if joint_name == "torso":
            _x = translation[0]
            _z = translation[2]

            side = "" # g:Jardin, d:Cour
            depth = ""
            _msg = ""

            left_margin = 0.1
            right_margin = 0.1

            distance_2m = 2.5
            distance_1m = 1.0

            if (_x < -left_margin):
                side = "cour"
            elif (_x > right_margin):
                side = "jardin"
            
            if (_z < -distance_2m):
                depth += "2m"
            elif (_z < -distance_1m):
                depth += "1m"

            _msg = side + "_" + depth
            if len(_msg) > 1:
                self.human_position_publisher.publish(_msg)
                print("x:", _x, "z:", _z, "[p : ", _msg, "]")

    def _broadcast_nuitrack_frame(self) -> None:
        """Broadcasts the camera location tf."""

        # Translation
        # NOTE : unit might be in meters
        x = 0
        y = 0
        z = 1
        translation = (x, y, z)

        # Rotation
        # Euler angles in degrees
        roll = -90.0
        pitch = 0.0
        yaw = 0.0

        # Convert degrees to radians
        roll_rad = radians(roll)
        pitch_rad = radians(pitch)
        yaw_rad = radians(yaw)

        # Convert radians to a quaternion
        rotation = transformations.quaternion_from_euler(
            roll_rad, pitch_rad, yaw_rad, axes="sxyz"
        )

        self.transform_broadcaster.sendTransform(
            translation, rotation, rospy.Time.now(),
            self.camera_frame, self.root_frame
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
