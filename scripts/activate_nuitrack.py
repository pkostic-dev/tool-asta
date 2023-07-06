#!/usr/bin/env python3

from PyNuitrack import py_nuitrack

REALSENSE = "Intel RealSense"
TRIAL = "Trial"
PRO = "Pro"

"""Activates a Nuitrack license on an Intel RealSense camera"""

if __name__ == "__main__":
    nuitrack = py_nuitrack.Nuitrack()
    nuitrack.init()

    print("Connect a single Intel RealSense camera and press Enter.")
    input()

    cameras = nuitrack.get_device_list()
    if cameras:
        first_camera = cameras[0]
        if REALSENSE in first_camera.get_name():
            msg = "Detected {name} [{serial_number}] : {activation}"
            print(
                msg.format(
                    name=first_camera.get_name(),
                    serial_number=first_camera.get_serial_number(),
                    activation=first_camera.get_activation(),
                )
            )

            activation_key = ""
            if TRIAL in first_camera.get_activation():
                print("Enter Nuitrack activation key : ")
                activation_key = input()
                if activation_key:
                    print("Activating . . .")
                    first_camera.activate(activation_key)
                    if PRO in first_camera.get_activation():
                        msg = "Succesfully activated {activation} licence."
                        print(
                            msg.format(activation=first_camera.get_activation())
                        )
            elif PRO in first_camera.get_activation():
                print("This camera has already been activated.")
        else:
            print(
                "Detected a camera that is not an Intel RealSense \
                         camera."
            )
    else:
        print("Didn't detect any cameras.")
