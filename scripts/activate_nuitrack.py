#!/usr/bin/env python3

from PyNuitrack import py_nuitrack

REALSENSE = "Intel RealSense"
TRIAL = "Trial"
PRO = "Pro"

if __name__ == "__main__":
    nuitrack = py_nuitrack.Nuitrack()
    nuitrack.init()

    print("Connect a single Intel RealSense camera and press Enter.", sep="")
    input()

    cameras = nuitrack.get_device_list()
    if cameras:
        first_camera = cameras[0]
        if REALSENSE in first_camera.get_name():
            print("Detected ",
                  first_camera.get_name(),
                  " [", first_camera.get_serial_number(), "] : ",
                  first_camera.get_activation(), sep="")
            
            activation_key = ""
            if TRIAL in first_camera.get_activation():
                print("Enter Nuitrack activation key : ", end="")
                activation_key = input()
                if activation_key:
                    print("Activating . . .")
                    first_camera.activate(activation_key)
                    if PRO in first_camera.get_activation():
                        print("Succesfully activated ",
                              first_camera.get_activation(),
                              " license.", sep="")
            elif PRO in first_camera.get_activation():
                print("This camera has already been activated.")
            
        else:
            print("Detected a camera that is not an Intel RealSense camera.")
    else:
        print("Didn't detect any cameras.")