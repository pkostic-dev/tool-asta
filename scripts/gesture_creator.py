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

DEBUG = False

def get_key(key_timeout:float) -> str:
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

        # SAVING
        self.save_manager = SaveManager()
        self.formats = ["json", "csv", "pkl"]
        self.format = 0

        # ROS
        rospy.init_node("gesture_creator", anonymous=True)
        # self.root_frame is set to the default root frame in ROS : "map"
        self.root_frame = "map"
        hz_rate = 10.0
        self.rospy_rate = rospy.Rate(hz_rate)
        self.transform_listener = tf.TransformListener()

        # JOINTS
        self.all_joints = [ "head", "neck", "torso", "waist", "left_collar",
                            "left_shoulder", "left_elbow", "left_wrist",
                            "left_hand", "right_collar", "right_shoulder",
                            "right_elbow", "right_wrist", "right_hand",
                            "left_hip", "left_knee", "left_ankle",
                            "right_hip", "right_knee", "right_ankle"]
        self.joints = ["torso", "head"] # used for saving joints
        l_elbow_angle = ["left_shoulder", "left_elbow", "left_wrist"]
        r_elbow_angle = ["right_shoulder", "right_elbow", "right_wrist"]
        self.joint_angles = [l_elbow_angle, r_elbow_angle] # for saving angles
        
        # INTERFACE
        self.key_timeout = 0.1
        self.interface_commands = {
            "save_joints": {
                "keys" : ["s", "S"],
                "name" : "Save joints",
                "action" : self.action_save_joints
            },
            "save_angles": {
                "keys" : ["a", "A"],
                "name" : "Save angles",
                "action" : self.action_save_angles
            },
            "load_file": {
                "keys" : ["l", "L"],
                "name" : "Load file",
                "action" : self.action_load
            },
            "change_format": {
                "keys" : ["c", "C"],
                "name" : "Change format",
                "action" : self.action_change_format
            },
            "exit": {
                "keys" : ["e", "E", "q", "Q"],
                "name" : "Exit",
                "action" : self.action_exit
            }
        }
        self.save_commands = {
            "snapshot": {
                "keys" : ["s", "S"],
                "name" : "Snapshot now",
                "action" : self.action_snapshot_save
            },
            "timer": {
                "keys" : ["t", "T"],
                "name" : "Timer set",
                "action" : self.action_timer_save
            },
            "cancel": {
                "keys" : ["c", "C", "e", "E", "q", "Q"],
                "name" : "Cancel",
                "action" : self.action_cancel
            }
        }
        self.current_commands = self.interface_commands

        self.print_format()
        self.print_lists()
        self.print_commands(self.current_commands)
    
    def print_format(self) -> None:
        print("Saving/Loading format is set to *.", self.formats[self.format])

    def print_lists(self) -> None:
        print("Joints list :", self.joints)
        print("Angles list :", self.joint_angles)

    def print_commands(self, commands:dict) -> None:
        if not commands:
            print("No commands available.")
            return
        print("Commands : ")
        msg = ""
        for command in commands:
            name = commands[command]["name"]
            key = commands[command]["keys"][0]
            msg += "[{key}] {name}    ".format(key=key, name=name)
        print(msg)

    def keyboard_callback(self, key:str) -> None:
        """
        Execute functions based on key pressed.
        """
        if DEBUG:
            print("Pressed [", key, "]", sep="")
        for command in self.current_commands:
            command_keys = self.current_commands[command]["keys"]
            if key in command_keys:
                action = self.current_commands[command]["action"]
                action()
                self.print_commands(self.current_commands)
                break

    def action_save_joints(self) -> None:
        self.current_commands = self.save_commands
        self.target = "joints"

    def action_save_angles(self) -> None:
        self.current_commands = self.save_commands
        self.target = "angles"

    def action_load(self) -> None:
        """
        Load sequence triggered by pressing 'l'.
        """

        print("Enter the file name : ", end='')
        file_name = input()
        print("Loading", file_name)
        data = self.load_file(file_name)
        if data:
            print(data)
    
    def action_change_format(self) -> None:
        if self.format + 1 <= len(self.formats) - 1:
            self.format += 1
        else:
            self.format = 0
        self.print_format()
    
    def action_exit(self) -> None:
        """
        SystemExit triggered by pressing 'e'.
        """

        print("Exiting...")
        raise SystemExit

    def action_snapshot_save(self) -> None:
        """
        Snapshot saving sequence triggered by pressing 's'.
        """
        data = {}
        if self.target == "joints":
            joints = self.joints
            print("Saving joints", joints)
            transformations = self.lookup_joints(joints)
            if not transformations:
                print_red("Couldn't look up joints.")
                return
            data = transformations

        if self.target == "angles":
            angles = self.joint_angles
            print("Saving angles", angles)
            calculated_angles = {}
            for joint_angle in angles:
                vertex = joint_angle[1]
                calculated_angles[vertex] = self.get_angle(joint_angle)
            data = calculated_angles

        print("Enter the file name : ", end="")
        file_name = input()
        print("Saving to", file_name)
        self.save_file(data, file_name)

    def action_timer_save(self) -> None:
        print("Enter timer duration : ", end="")
        duration = int(input())
        while duration > 0:
            print(str(duration) + "sec left")
            rospy.sleep(1)
            duration -= 1
        print("Cheese")
        self.action_snapshot_save()

    def action_cancel(self) -> None:
        self.current_commands = self.interface_commands

    def get_angle(self, joints:list) -> dict:
        """
        Calculates the angle of the joints in list. Returns a dictionary
        that contains joint keys, list of joints, and angle in degrees. 
        """
        
        result = {}
        transformations = self.lookup_joints(joints)
        result["joint_keys"] = list(transformations.keys())
        result["joints"] = transformations
        rotations = []
        for _, value in transformations.items():
            rotation = value[1]
            rotations.append(rotation)
        degrees = calculate_degrees(*rotations)
        result["degrees"] = degrees

        return result

    def lookup_joints(self, joints:list, ids=[0]) -> dict:
        """
        Looks up the joints in the list and returns a dictionary with
        each joint as key and the translation and rotation arrays as values.
        """

        result = {}
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

    def save_file(self, data:dict, file_name:str) -> None:
        if self.formats[self.format] == "json":
            self.save_manager.save_dict_to_json(data, file_name)
        elif self.formats[self.format] == "csv":
            self.save_manager.save_dict_to_csv(data, file_name)
        elif self.formats[self.format] == "pkl":
            self.save_manager.save_dict_to_pickle(data, file_name)
        else:
            self.save_manager.save_any(data, file_name)

    def load_file(self, file_name:str) -> dict:
        data = self.save_manager.load_any(file_name)
        return data

    def update(self) -> None:
        """
        Update loop. Checks for key presses.
        """

        key = get_key(self.key_timeout)
        if len(key):
            self.keyboard_callback(key)
        self.rospy_rate.sleep()

    def launch(self) -> None:
        """
        Continously update. Handle exceptions.
        """

        while not rospy.is_shutdown():
            try:
                self.update()
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise SystemExit


if __name__ == "__main__":
    GC = GestureCreator()
    GC.launch()