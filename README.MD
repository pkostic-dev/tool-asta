# Skeleton Converter

Converts the skeleton data obtained via the Nuitrack SDK (PyNuitrack) into transform data and publishes it to ROS via tf.TransformBroadcaster.

## Dependencies

This module requires the workspace to be built using Python 3.6. This is a list of all Python 3 modules you will need :

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
`rosrun skeleton_converter skeleton_converter.py`
