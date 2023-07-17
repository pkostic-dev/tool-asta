#!/usr/bin/env python3

import termios
import sys
import select
import tty
import pickle
from datetime import datetime

import rospy
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
        update_frequency = 10.0  # NOTE : in Hz
        self.rospy_rate = rospy.Rate(update_frequency)
        self.transform_listener = TransformListener()
        self.joints = ["torso"]

        print("Usage :")
        print("  [ Save : s ]    [ Load : l ]    [ Exit : e ]  ")
    
    def _update(self) -> None:
        key = get_key(0.1)
        if (len(key)):
            print("Pressed key : " + key)
            self._keyboard_callback(key)
        self.rospy_rate.sleep()

    def _keyboard_callback(self, key) -> None:
        if key == 's':
            self._save()

        if key == 'l':
            self._load()

        if key == 'e':
            self._exit()

    def _lookup_joints(self, joints, ids=[0]) -> dict:
        result = dict()
        for id in ids:
            for joint in joints:
                joint_name = joint + "_" + str(id)
                (tslt, rttn) = self.transform_listener.lookupTransform(
                    joint_name, self.root_frame, rospy.Time(0))
                result[joint] = (tslt, rttn)
        return result

    def _save(self) -> None:
        print("Saving...")
        joints = self._lookup_joints(self.joints)

        print("Enter the file name : ", end="")
        file_name = input()
        if not file_name:
            now = datetime.now()
            now_str = now.strftime("-%Y-%m-%d-%H-%M-%S")
            file_name = 'joints' + now_str
        if file_name[-4:] != '.pkl':
            file_name += '.pkl'

        print("Saving to", file_name)

        self._save_joints(joints, file_name)

    def _load(self) -> None:
        print("Loading...")
        print("Enter the file name : ", end='')
        file_name = input()
        if file_name[-4:] != '.pkl':
            file_name += '.pkl'
        print("Loading", file_name)
        print(self._load_joints(file_name))

    def _exit(self) -> None:
        print("Exiting...")
        raise SystemExit

    def _save_joints(self, joints, file_name) -> None:
        with open(file_name, 'wb') as file:
            pickle.dump(joints, file)
        file.close()

    def _load_joints(self, file_name = 'joints.pkl') -> dict:
        joints = dict()
        with open(file_name, 'rb') as file:
            joints = pickle.load(file)
        file.close()
        return joints

    def launch(self) -> None:
        while not rospy.is_shutdown():
            try:
                self._update()
            except (LookupException,
                ConnectivityException,
                ExtrapolationException) as exception:
                pass
            except FileNotFoundError:
                print("File not found.")
            except KeyboardInterrupt:
                rospy.signal_shutdown("KeyboardInterrupt")
                raise SystemExit


if __name__ == "__main__":
    GC = GestureCreator()
    GC.launch()