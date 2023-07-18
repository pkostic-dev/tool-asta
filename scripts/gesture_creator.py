#!/usr/bin/env python3

import termios
import sys
import select
import tty
import pickle
from datetime import datetime

import rospy
from tf import TransformListener, LookupException, ExtrapolationException, ConnectivityException

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
        self.transform_listener = TransformListener()
        self.all_joints = [ "head", "neck", "torso", "waist", "left_collar",
                            "left_shoulder", "left_elbow", "left_wrist",
                            "left_hand", "right_collar", "right_shoulder",
                            "right_elbow", "right_wrist", "right_hand",
                            "left_hip", "left_knee", "left_ankle",
                            "right_hip", "right_knee", "right_ankle"]
        self.joints = ["torso"]

        print("Usage :")
        print("  [ Save : s ]    [ Load : l ]    [ Exit : e ]  ")
    
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

        if key == 'e' or key == 'E':
            self._exit()

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

    def _save(self) -> None:
        """
        Save sequence triggered by pressing 's'.
        """

        print("Saving...")
        joints = self._lookup_joints(self.joints)
        print("Enter the file name : ", end="")
        file_name = input()
        print("Saving to", file_name)
        self.save_manager.save_pickle(joints, file_name)

    def _load(self) -> None:
        """
        Load sequence triggered by pressing 'l'.
        """

        print("Loading...")
        print("Enter the file name : ", end='')
        file_name = input()
        print("Loading", file_name)
        print(self.save_manager.load_pickle(file_name))

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
            except (LookupException,
                ConnectivityException,
                ExtrapolationException) as exception:
                pass
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise SystemExit


if __name__ == "__main__":
    GC = GestureCreator()
    GC.launch()