#!/usr/bin/env python3

import logging
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

        self.nuitrack = py_nuitrack.Nuitrack()

        # TransformBroadcaster initialisation
        logging.info("Initialising tf.TransformBroadcaster object.")
        self.transform_broadcaster = tf.TransformBroadcaster()

        # Publisher of human gestures
        self.human_gesture_publisher = rospy.Publisher(
            'human_gesture',
            String,
            queue_size=1
        )

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

        self.init_nuitrack()

        rospy.init_node("tf_skeletons", anonymous=True)

        update_frequency = 1.0 # in Hz
        self.rospy_rate = rospy.Rate(update_frequency)

    def init_nuitrack(self) -> None:
        """Initialises a Nuitrack object.
        
        Initialises a Nuitrack object. Searches for a Nuitrack compatible device
        and selects the first one. Checks the activation status of the license.
        Finally the Nuitrack object is run.
        """
        
        logging.info("Initialising py_nuitrack.Nuitrack object.")
        self.nuitrack.init()

        logging.info("Getting Nuitrack device list...")
        devices = self.nuitrack.get_device_list()
        for i, dev in enumerate(devices):
            logging.info("Device %d:\
                \n\tname = %s,\
                \n\tserial_number = %s",
                i,
                dev.get_name(),
                dev.get_serial_number())
            if i == 0:
                if dev.get_activation() == "None":
                    logging.warn("No active license for device %d.", i)
                    self.nuitrack.release()
                    return
                
                # If a license hasn't been activated with the Nuitrack Runtime
                # package it can be activated using Python API
                # dev.activate("ACTIVATION_KEY")

                logging.info("Device %d activation status : %s",
                     i,
                    dev.get_activation())
                logging.info("Setting device %d as active.", i)
                self.nuitrack.set_device(dev)

        logging.info("Creating Nuitrack modules.")
        self.nuitrack.create_modules()
        self.nuitrack.run()
        logging.info("Nuitrack is running.")

        # Set up a shutdown hook for releasing Nuitrack from memory.
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self) -> None:
        """Shutdown hook for the Nuitrack object."""

        logging.info("Shutting down nuitrack.")
        if self.nuitrack != None:
            self.nuitrack.release()

    def launch(self) -> None:
        """Continuously update Nuitrack, get skeleton data, and load it."""

        try:
            while not rospy.is_shutdown():
                self.nuitrack.update()
                data = self.nuitrack.get_skeleton()
                self.store_skeletons(data)
                self.broadcast_skeletons()
                self.rospy_rate.sleep()
        except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise SystemExit
    
    def store_skeletons(self, data) -> None:
        """Store the skeletons data from Nuitrack."""

        # Loop through all skeletons found
        for skeleton in range(0,len(data.skeletons)):
            # Verify if the detected skeleton's user_id has already been stored
            if data.skeletons[skeleton].user_id in self.ids:
                # If the skeleton has been detected then overwrite its joints data
                id = self.ids.index(data.skeletons[skeleton].user_id)
                self.store_joints(id)
                self.joints[id] = data.skeletons[skeleton][1:]
            else:
                # if not, we will add now id and save data in the joints list,
                # translation list and rotation list
                self.ids.append(data.skeletons[skeleton].user_id)
                id = len(self.ids)-1
                self.joints.append(data.skeletons[skeleton][1:])

    def store_joints(self, id) -> None:
        for joint in range(0,20):
            self.translation[id,joint,:] = \
                (-np.array(self.joints[id][joint].real)/1000.0).tolist() 
            self.rotation[id,joint,:] = \
                self.joints[id][joint].orientation.flatten()

    def broadcast_skeletons(self) -> None:
        """Function send Transform information of every point of skeletons"""

        for skeleton_id in self.ids:
            id = self.ids.index(skeleton_id)

            # Calculate the rotation in Euler angles. [joint, xyz]
            euler_rotations = np.zeros([20, 3])

            for joint_number in range(20): 
                r = self.rotation[id, joint_number, :]
                m = np.mat([
                    [r[0], r[1], r[2]],
                    [r[3], r[4], r[5]],
                    [r[6], r[7], r[8]]
                ])
                euler_rotation = tf.transformations.euler_from_matrix(m, "rxyz")
                euler_rotations[joint_number, :] = euler_rotation
            
            # Broadcast /nuitrack_frame position and rotation
            self.broadcast_nuitrack_frame()

            joints_moving = np.ones(20, dtype=bool)
            # Send transform message for each joint
            for joint_number in range(20):
                translation = self.translation[id,joint_number, :]
                rotation = tf.transformations.quaternion_from_euler(
                    euler_rotations[joint_number, 0],
                    euler_rotations[joint_number, 1],
                    euler_rotations[joint_number, 2])
                time = rospy.Time.now()
                child = str(self.joints[id][joint_number].type)+"_"+str(id)
                parent = "/nuitrack_frame"

                # Triggers based on depth and lateral position
                # z => profondeur ; x => lateral
                # if torso
                if (child == "torso_%d" % id):
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

                    if (_x < -0):
                        _msg = "human_%d_left" % id
                    elif (_x > 0):
                        _msg = "human_%d_right" % id
                    
                    if (_z < -3.0):
                        _msg += "_2m"
                    elif (_z < -1.0):
                        _msg += "_1m"

                    if _msg:
                        pass
                        #self.human_gesture_publisher.publish(_msg)
                        print(_msg)
                        print("\t", _x, _z)

                if not (translation == self.sent_translation[id, joint_number, :]).all():
                    if self.present_ids[id] == False:
                        self.present_ids[id] = True
                    self.transform_broadcaster.sendTransform(
                        translation,
                        rotation,
                        time,
                        child,
                        parent)
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

    def broadcast_nuitrack_frame(self) -> None:
        """Broadcasts nuitrack frame (camera position, manual configuration)."""
        
        x = 0
        y = 0
        z = 1
        position_nuitrack = (x, y, z) # NOTE : unit ?

        # Euler angles in degrees
        roll  = -90.0
        pitch = 0.0
        yaw   = 0.0

        # Convert degrees to radians
        roll_rad  = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad   = math.radians(yaw)

        # Convert radians to a quaternion
        rotation_nuitrack = tf.transformations.quaternion_from_euler(
            roll_rad,
            pitch_rad,
            yaw_rad,
            axes='sxyz'
        )
        
        # Broadcast
        self.transform_broadcaster.sendTransform(
            position_nuitrack,
            rotation_nuitrack,
            rospy.Time.now(),
            "nuitrack_frame",
            "map"
        )


if __name__ == '__main__':
    # Configure logging module
    logging.basicConfig(level=logging.DEBUG)

    SC = SkeletonConverter()
    SC.launch()