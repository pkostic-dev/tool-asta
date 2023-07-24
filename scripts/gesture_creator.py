#!/usr/bin/env python3

import termios
import sys
import select
import tty
import os

import rospy
import tf

from save_manager import SaveManager
from helper import calculate_degrees, print_green, print_red


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
        # self.root_frame is set to the default root frame in ROS : "map"
        self.root_frame = "map"
        hz_rate = 10.0
        self.rospy_rate = rospy.Rate(hz_rate)
        self.transform_listener = tf.TransformListener()
        self.all_joints = [ "head", "neck", "torso", "waist", "left_collar",
                            "left_shoulder", "left_elbow", "left_wrist",
                            "left_hand", "right_collar", "right_shoulder",
                            "right_elbow", "right_wrist", "right_hand",
                            "left_hip", "left_knee", "left_ankle",
                            "right_hip", "right_knee", "right_ankle"]
        self.formats = ["json", "csv", "pickle"]
        self.format = 0

        self.joints = ["torso", "head"] # used for saving joints
        l_elbow_angle = ["left_shoulder", "left_elbow", "left_wrist"]
        r_elbow_angle = ["right_shoulder", "right_elbow", "right_wrist"]
        self.joint_angles = [l_elbow_angle, r_elbow_angle] # for saving angles
        
        self.commands = {"s": "Save joints", "a": "Save angles", 
                         "l": "Load file", "c": "Change format", "e": "Exit"}
        self._print_commands()
    
    def _print_commands(self):
        print()
        print("Saving/Loading format :", self.formats[self.format])
        print("Joints list :", self.joints)
        print("Angles list :", self.joint_angles)
        print("Commands : ")
        msg = ""
        for key in self.commands:
            command = self.commands[key]
            msg += "[{key}] {command}".format(key=key, command=command)
            msg += "    "
        print(msg)

    def _update(self) -> None:
        """
        Update loop. Checks for key presses.
        """

        key = get_key(0.1)
        if (len(key)):
            print("Pressed [", key, "]", sep="")
            self._keyboard_callback(key)
        self.rospy_rate.sleep()

    def _keyboard_callback(self, key) -> None:
        """
        Execute functions based on key pressed.
        """
        # FIXME: couple with self.commands
        if key == 's' or key == 'S':
            self._save_joints(self.joints)

        if key == 'a' or key == 'A':
            self._save_angles(self.joint_angles)

        if key == 'l' or key == 'L':
            self._load()

        if key == 'c' or key == 'C':
            self._change_format()

        if key == 'e' or key == 'E':
            self._exit()
        
        self._print_commands()

    def _lookup_joints(self, joints, ids=[0]) -> dict:
        """
        Looks up the joints in the list and returns a dictionary with
        each joint as key and the translation and rotation arrays as values.
        """

        result = dict()
        try:
            for id in ids:
                for joint in joints:
                    joint_name = joint + "_" + str(id)
                    (tslt, rttn) = self.transform_listener.lookupTransform(
                        joint_name, self.root_frame, rospy.Time(0))
                    result[joint] = [tslt, rttn]
        except (tf.LookupException,
                tf.ConnectivityException,
                tf.ExtrapolationException) as exception:
                print(exception)

        return result

    def _get_angle(self, joints) -> dict:
        """
        Calculates the angle of the joints in list. Returns a dictionary
        that contains joint keys, list of joints, and angle in degrees. 
        """
        
        result = {}
        transformations = self._lookup_joints(joints)
        result["joint_keys"] = list(transformations.keys())
        result["joints"] = transformations
        rotations = []
        for _, value in transformations.items():
            rotation = value[1]
            rotations.append(rotation)
        degrees = calculate_degrees(*rotations)
        result["degrees"] = degrees

        return result

    def _save_joints(self, joints) -> None:
        """
        Joint saving sequence triggered by pressing 's'.
        """

        print("Saving joints", joints)
        transformations = self._lookup_joints(joints)
        if not transformations:
            print_red("Couldn't look up joints.")
            return
        print("Enter the file name : ", end="")
        file_name = input()
        print("Saving to", file_name)
        if self.formats[self.format] == "json":
            self.save_manager.save_dict_to_json(transformations, file_name)
        elif self.formats[self.format] == "csv":
            self.save_manager.save_dict_to_csv(transformations, file_name)
        elif self.formats[self.format] == "pickle":
            self.save_manager.save_dict_to_pickle(transformations, file_name)
        else:
            self.save_manager.save_any(transformations, file_name)

    def _save_angles(self, angles) -> None:
        """
        Angle saving sequence triggered by pressing 'a'.
        """

        print("Saving angles", angles)

        calculated_angles = {}
        for joint_angle in angles:
            vertex = joint_angle[1]
            calculated_angles[vertex] = self._get_angle(joint_angle)

        print("Enter the file name : ", end="")
        file_name = input()
        print("Saving to", file_name)
        if self.formats[self.format] == "json":
            self.save_manager.save_dict_to_json(calculated_angles, file_name)
        elif self.formats[self.format] == "csv":
            self.save_manager.save_dict_to_csv(calculated_angles, file_name)
        elif self.formats[self.format] == "pickle":
            self.save_manager.save_dict_to_pickle(calculated_angles, file_name)
        else:
            self.save_manager.save_any(calculated_angles, file_name)

    def _load(self) -> None:
        """
        Load sequence triggered by pressing 'l'.
        """

        print("Enter the file name : ", end='')
        file_name = input()
        print("Loading", file_name)
        data = {}
        if self.formats[self.format] == "json":
            data = self.save_manager.load_json_to_dict(file_name)
        elif self.formats[self.format] == "csv":
            data = self.save_manager.load_csv_to_dict(file_name)
        elif self.formats[self.format] == "pickle":
            data = self.save_manager.load_pickle_to_dict(file_name)
        else:
            data = self.save_manager.load_any(file_name)
        if data:
            print(data)
    
    def _change_format(self) -> None:
        if self.format + 1 <= len(self.formats) - 1:
            self.format += 1
        else:
            self.format = 0
    
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
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise SystemExit


if __name__ == "__main__":
    GC = GestureCreator()
    GC.launch()