# Asta Tool

Asta Tool is a set of scripts written in Python 3.6 which are designed to address two distinct domains: assisting children with directional dyslexia and enhancing theater experiences with social robots like Nao and QTRobot. The software provides the terminal interface GestureCreator which enables users to save joints and angles from percieved skeleton data to save predefined gestures and positions tailored to individual theatrical or learning needs. The core functionality is managed by the GestureDetector which allows users to define areas, body positions, and other rules as triggers within a virtual space. The triggers are published on a ROS topic, allowing them to be listened to by any other machine connected to the same ROS core.

## SkeletonConverter

The `skeleton_converter.py` script converts the skeleton data obtained via the Nuitrack SDK (PyNuitrack) into transform (tf) data and publishes it to ROS via the `tf.TransformBroadcaster()` class. The script's node is called `tf_skeletons`. It broadcasts the tf data on the root frame 'map', the default root frame name in rviz. The default broadcasting rate is set at 10Hz. At initialization, it will check whether the depth camera detected by Nuitrack SDK has an activated license. It doesn't publish skeleton data that has less than 0.5 confidence (currently confidence is either 0.0 or 0.75 because it is not yet fully implemented by the Nuitrack team). It also doesn't publish the same skeleton data twice, because sometimes invisible people may appear as detected with 0.75 confidence. 

## Activate Nuitrack

The `activate_nuitrack.py` script allows for easy activation of a Nuitrack license on a Intel RealSense camera.

## Helper

The `helper.py` script contains shared functions that are used across other scripts, providing utility
functions and common functionalities.

## SaveManager

The `save_manager.py` script contains a class of the same name that is used for saving and loading joints in multiple different formats. Available formats are : pickle, json, and csv. It saves to and loads from the working directory.

## GestureCreator

Allows the creation and loading of select joints and angles using a simple terminal interface.

## GestureDetector

Can detect angles between 3 joints, and set up trigger around a circle. The circle trigger is set up around a given joint and radius. It then publishes messages to ROS on trigger.

## Dependencies

This toolset requires the workspace to be built using Python 3.6. This is a list of all Python 3 modules you will need :

- math
- PyNuitrack
- rospy
- tf
- numpy

## Executing the script

Create a symbolic link to your workspace's `src` folder :  
`ln -s <path_to_git_folder> <path_to_workspace>/src`

Make the scripts executable :  
`chmod +x *.py`

You need to build the workspace using the following command :  
`catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so`

Don't forget to `source` the `setup.sh` file :  
`source devel/setup.sh`

Run using `rosrun` :  

- Connect your Intel RealSense camera.
- Then run the skeleton_converter : `rosrun skeleton_converter skeleton_converter.py`
- Finally run one of the other tools :
  - GestureCreator : `rosrun skeleton_converter gesture_creator.py`
  - GestureDetector : `rosrun skeleton_converter gesture_detector.py`
