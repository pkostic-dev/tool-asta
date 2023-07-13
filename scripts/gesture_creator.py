#!/usr/bin/env python3

import termios
import sys
import select
import tty
import pickle

import rospy
import rospkg
from tf import TransformListener, LookupException, ExtrapolationException, ConnectivityException, transformations


def get_key(key_timeout):
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
    def __init__(self) -> None:
        super().__init__()

        rospy.init_node("gesture_creator", anonymous=True)
        self.root_frame = "map" # NOTE : root frame, might have to change it
        update_frequency = 1.0  # NOTE : in Hz
        self.rospy_rate = rospy.Rate(update_frequency)
        self.transform_listener = TransformListener()
        self.joints = ["torso"]
    
    def _update(self) -> None:
        key = get_key(0.1)
        if (len(key)):
            print("Pressed a key")
            self._keyboard_callback(key)
        self.rospy_rate.sleep()

    def _keyboard_callback(self, key) -> None:
        if key == 's':
            print("Pressed 's'")
            self._save_joints()
        if key == 'l':
            print("Pressed 'l'")
            self._load_joints()

    def _lookup_joints(self, joints, ids=[0]) -> list:
        result = []
        for id in ids:
            for joint in joints:
                joint_name = joint + "_" + str(id)
                (tslt, rttn) = self.transform_listener.lookupTransform(
                    joint_name, self.root_frame, rospy.Time(0))
                result.append((tslt, rttn))
        return result

    def _save_joints(self):
        print("Looking up ", self.joints)
        transformations = self._lookup_joints(self.joints)
        print("Got joints : ", transformations)

        file_name = 'joints.pkl'
        with open(file_name, 'wb') as file:
            pickle.dump(transformations, file)
        file.close()

    def _load_joints(self):
        transformations = []
        file_name = 'joints.pkl'
        with open(file_name, 'rb') as file:
            transformations = pickle.load(file)
        file.close()
        print(transformations)

    def launch(self) -> None:
        try:
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