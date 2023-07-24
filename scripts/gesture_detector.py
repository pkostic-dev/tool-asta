#!/usr/bin/env python3

import time
import pprint

import rospy

from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException # type: ignore
from std_msgs.msg import String
from save_manager import SaveManager
from helper import calculate_degrees, calculate_distances, is_point_inside_circle, translation_average, print_red, print_green


class GestureDetector():
    """Can detect angles between 3 joints, and set up trigger around a circle.

    The circle trigger is set up around a given joint and radius.
    """
    
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node("gesture_detector", anonymous=True)

        self.save_manager = SaveManager()

        # Publisher of human gestures
        self.human_gesture_publisher = rospy.Publisher(
            'human_gesture',
            String,
            queue_size=1
        )

        self.root_frame = "map" # NOTE : root frame, might have to change it
        update_frequency = 10.0  # NOTE : in Hz
        self.rospy_rate = rospy.Rate(update_frequency)
        self.time = 0
        self.gesture_publisher = rospy.Publisher(
            'gesture', String, queue_size=1
            )
        self.transform_listener = TransformListener()
        self.tfs = []

        self.all_joints = ["head", "neck", "torso", "waist", "left_collar",
                       "left_shoulder", "left_elbow", "left_wrist",
                       "left_hand", "right_collar", "right_shoulder",
                       "right_elbow", "right_wrist", "right_hand",
                       "left_hip", "left_knee", "left_ankle",
                       "right_hip", "right_knee", "right_ankle"]
        
        self.treasure = self.save_manager.load_pickle_to_dict('treasure.pkl')
        self.goal = self.treasure
        
        # Set up distance check to goal
        self.distance_timer:rospy.Timer
        duration = 2 # NOTE : shorter for debugging purpose
        self._timed_distance_checker(self.treasure, duration,
                                     self._distance_callback)

    def _timed_distance_checker(self, target_joint, duration, callback) -> None:
        """
        Sets up a timer that checks the distance to goal. Sets goal.
        """

        self.goal = target_joint
        self.distance_timer = rospy.Timer(rospy.Duration(duration), callback)

    def _distance_callback(self, _) -> None:
        """
        Checks distance from goal.
        """

        # Check distance from the treasure point
        (dist_x, dist_y, _) = self._check_distance(self.goal)
        if dist_x > 0.3:
            print("dist_x > 0.3")
        if dist_x < 0.3:
            print("dist_x < 0.3")
        if dist_y > 0.3:
            print("dist_y > 0.3")
        if dist_y < 0.3:
            print("dist_y < 0.3")
        # self.human_gesture_publisher.publish(distance)

    def _get_joint_points(self, target_joint) -> tuple:
        target_joint_name = list(target_joint.keys())[0]
        joint = self._lookup_joints([target_joint_name])

        target_x = target_joint[target_joint_name][0][0]
        target_z = target_joint[target_joint_name][0][2]
        real_x = joint[target_joint_name][0][0]
        real_z = joint[target_joint_name][0][2]

        return (target_x, target_z, real_x, real_z)

    def _check_distance(self, target_joint:dict) -> tuple:
        (center_x, center_z, point_x, point_z) = self._get_joint_points(
            target_joint)

        return calculate_distances(center_x, center_z, point_x, point_z)

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
    
    def _get_angle(self, joints) -> float:
        """
        Calculates the angle of the joints in list. Returns degrees.
        """

        transformations = self._lookup_joints(joints)
        rotations = []
        for _, value in transformations.items():
            rotation = value[1]
            rotations.append(rotation)
        angle = calculate_degrees(*rotations)
        return angle

    def _get_average(self) -> list:
        """
        Calculates the average of all joints. Returns average translation"""

        transformations = self._lookup_joints(self.all_joints)
        average = translation_average(transformations)

        return average

    def _get_joints(self, joints) -> None:
        """
        Pretty prints the joints in list.
        """

        jnts = self._lookup_joints(joints)
        pprint.pprint(jnts)

    def _circle_trigger(self, target_joint:dict, trigger:str,
                        rad:float=0.5) -> None:
        """
        Sets up a circle trigger given a target joint around which the circle
        is set up using the radius. The joints name is used to look up the
        realtime joint. If the realtime joint is in target joint circle,
        trigger is published.
        """

        (center_x, center_z, point_x, point_z) = self._get_joint_points(
            target_joint)

        if is_point_inside_circle(center_x, center_z, rad, point_x, point_z):
            print("In circle")
            self.human_gesture_publisher.publish(trigger)

    def _update(self) -> None:
        """
        Update loop.
        """

        self.time = time.asctime(time.localtime(time.time()))

        # all_joints = self._lookup_joints(self.all_joints)
        # print(all_joints)
        print(self._get_angle(["left_wrist", "left_elbow", "left_shoulder"]))
        # self._get_joints(["torso"])

        #self._circle_trigger(self.treasure, "treasure_found", 0.1)

        self.rospy_rate.sleep()

    def launch(self) -> None:
        """Continuously update."""

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


""" TODO
TODO verify if angles are correct
    -> testing
TODO angles publisher
    -> easy
TODO remake the threshold but with a rectangle not a line
    -> easy
TODO other easy shapes ? (ellipse)
    -> ask chat GPT
TODO threshold for 3D positions (+height) for lifting arm etc.
    -> easy ?
TODO add basic gestures & publisher
    -> time consuming, need another person
    -> T pose
    -> mains en air (1 our 2, G ou D)
TODO average of all points

TODO add multiple goals, goal switching
TODO stop distance check publisher when at goal (?)

TODO : detect sequence of gestures
    -> first detect initial gesture, set flag
    -> set a timer
    -> check if next gesture is validated
    -> publish

"""

if __name__ == "__main__":
    GD = GestureDetector()
    GD.launch()