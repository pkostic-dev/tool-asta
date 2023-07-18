#!/usr/bin/env python3

import termios
import sys
import select
import tty
import pickle
from datetime import datetime

import rospy
import tf

from save_manager import SaveManager


def get_key(key_timeout):
    """
    Check for key presses during a timeout period.
    """

    file_descriptor = sys.stdin.fileno()
    old = termios.tcgetattr(file_descriptor)

    tty.setraw(file_descriptor)

    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)

    key = ''
    if rlist:
        key = sys.stdin.read(1)
        
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)

    return key


class GestureCreator():
    """Allows the creation and loading of joints using simple terminal 
    interface"""

    def __init__(self) -> None:
        super().__init__()

        self.save_manager = SaveManager()
        rospy.init_node("gesture_creator", anonymous=True)
        self.root_frame = "map" # NOTE : root frame, might have to change it
        update_frequency = 10.0  # NOTE : in Hz
        self.rospy_rate = rospy.Rate(update_frequency)
        self.transform_listener = tf.TransformListener()
        self.all_joints = [ "head", "neck", "torso", "waist", "left_collar",
                            "left_shoulder", "left_elbow", "left_wrist",
                            "left_hand", "right_collar", "right_shoulder",
                            "right_elbow", "right_wrist", "right_hand",
                            "left_hip", "left_knee", "left_ankle",
                            "right_hip", "right_knee", "right_ankle"]
        self.joints = ["torso"]
        self.format = "json"
        self.commands = "[s] Save    [l] Load    [t] Toggle format    [e] Exit"
        print("Saving/Loading format :", self.format)
        print(self.commands)
    
    def _update(self) -> None:
        """
        Update loop. Checks for key presses.
        """

        key = get_key(0.1)
        if (len(key)):
            print("Pressed key : " + key)
            self._keyboard_callback(key)
        self.rospy_rate.sleep()

    def _keyboard_callback(self, key) -> None:
        """
        Execute functions based on key pressed.
        """

        if key == 's' or key == 'S':
            self._save()

        if key == 'l' or key == 'L':
            self._load()

        if key == 't' or key == 'T':
            self._toggle_format()

        if key == 'e' or key == 'E':
            self._exit()
        
        print(self.commands)

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
                result[joint] = [tslt, rttn]

        return result

    def _toggle_format(self) -> None:
        if self.format == "json":
            self.format = "csv"
            print("Saving/Loading format :", self.format)
        elif self.format == "csv":
            self.format = "json"
            print("Saving/Loading format :", self.format)

    def _save(self) -> None:
        """
        Save sequence triggered by pressing 's'.
        """

        print("Saving...")
        joints = self._lookup_joints(self.joints)
        print("Enter the file name : ", end="")
        file_name = input()
        print("Saving to", file_name)
        if self.format == "json":
            self.save_manager.save_json(joints, file_name)
        elif self.format == "csv":
            self.save_manager.save_csv(joints, file_name)

    def _load(self) -> None:
        """
        Load sequence triggered by pressing 'l'.
        """

        print("Enter the file name : ", end='')
        file_name = input()
        print("Loading", file_name)
        joints = {}
        if self.format == "json":
            joints = self.save_manager.load_json(file_name)
        if self.format == "csv":
            joints = self.save_manager.load_csv(file_name)
        if joints:
            print(joints)

    def _exit(self) -> None:
        """
        SystemExit triggered by pressing 'e'.
        """

        print("Exiting...")
        raise SystemExit

    def launch(self) -> None:
        """
        Continously update. Handle exceptions.
        """

        while not rospy.is_shutdown():
            try:
                self._update()
            except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as exception:
                pass
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise SystemExit


if __name__ == "__main__":
    GC = GestureCreator()
    GC.launch()