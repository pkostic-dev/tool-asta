#!/usr/bin/env python3

joints = [ "head", "neck", "torso", "waist", "left_collar","left_shoulder", 
           "left_elbow", "left_wrist","left_hand", "right_collar", 
           "right_shoulder","right_elbow", "right_wrist", "right_hand",
           "left_hip", "left_knee", "left_ankle","right_hip", "right_knee", 
           "right_ankle"]
angles = {
    "l_elbow" : ["left_shoulder", "left_elbow", "left_wrist"],
    "r_elbow" : ["right_shoulder", "right_elbow", "right_wrist"],
    "l_shoulder" : ["left_hip", "left_shoulder", "left_elbow"],
    "r_shoulder" : ["right_hip", "right_shoulder", "right_elbow"]
}