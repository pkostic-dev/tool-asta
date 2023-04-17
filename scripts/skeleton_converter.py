#!/usr/bin/env python3
import rospy
import tf
from PyNuitrack import py_nuitrack
import numpy as np
import logging


class SkeletonConverter:
	"""Converts nuitrack skeleton data into tf data and publishes it.

	This class creates initialises a Nuitrack object. Converts the data from it
	into TF skeleton data. And publishes the TF skeleton data to ROS with a
	TransformBroadcaster object.
	"""

	def __init__(self):
		"""
		Initialisation of class, creates Nuitrack and TransformBroadcaster
		objects.
		"""

		super().__init__()

		self.nuitrack = py_nuitrack.Nuitrack()

		# TransformBroadcaster initialisation
		logging.info("Initialising tf.TransformBroadcaster object.")
		self.transform_broadcaster = tf.TransformBroadcaster()

		# The ids of percieved skeletons
		self.ids = []
		# List containing joints of each skeleton. The length will be 20.
		# Because Nuitrack describe skeleton with 20 joints. Every joint item
		# has information of translation and rotation
		self.joints = []
		# The translations of every joint in space [id, joint, xyz]
		self.translation = np.zeros([10, 25, 3])
		# The rotation of every joint in space [id, joint, matrix]
		self.rotation = np.zeros([10, 25, 9])


	def init_nuitrack(self) -> None:
		"""Initialises a Nuitrack object.
		
		This method initialises a Nuitrack object. It searches for a Nuitrack
		compatible device and selects the first one. It checks the activation
		status of the license. Finally it runs the Nuitrack object.
		"""
		
		logging.info("Initialising py_nuitrack.Nuitrack object.")
		self.nuitrack.init()

		logging.info("Getting Nuitrack device list...")
		devices = self.nuitrack.get_device_list()
		for i, dev in enumerate(devices):
			logging.info("Device %d:\
				\n\tname = %s,\
				\n\tserial_number = %s",
				i,
				dev.get_name(),
				dev.get_serial_number())
			if i == 0:
				if dev.get_activation() == "None":
					logging.warn("No active license for device %d.", i)
					self.nuitrack.release()
					return
				
				# If a license hasn't been activated with the Nuitrack Runtime
				# package it can be activated using Python API
				# dev.activate("ACTIVATION_KEY")

				logging.info("Device %d activation status : %s",
		 			i,
					dev.get_activation())
				logging.info("Setting device %d as active.", i)
				self.nuitrack.set_device(dev)

		logging.info("Creating Nuitrack modules.")
		self.nuitrack.create_modules()
		self.nuitrack.run()
		logging.info("Nuitrack is running.")

		# Set up a shutdown hook for releasing Nuitrack from memory.
		rospy.on_shutdown(self.shutdown_hook)


	def shutdown_hook(self):
		logging.info("Shutting down nuitrack.")
		self.nuitrack.release()


	def update(self):
		"""Continuously update Nuitrack, get, convert, and load skeleton data."""

		while not rospy.is_shutdown():
			self.nuitrack.update()
			data = self.nuitrack.get_skeleton()
			self.load(data)

	
	def load(self,data):
		"""
		Function for load data heard from nuitrack	

		Args:
		data: The data heard from nuitrack
		"""

		# load data of every id 
		for skeleton in range(0,len(data.skeletons)):
			# verify if this id was detected before
			
			if data.skeletons[skeleton].user_id in self.ids:
				# if id was detected before, we copy the joint's previous
				# translation and rotation to the translation list and rotation
				# list.
				# Then Overwrite the original data of joints with the new state 
				id = self.ids.index(data.skeletons[skeleton].user_id)

				for joint in range(0,20):
					# NOTE : find out why divided by 1000.0 and why negative ?
					self.translation[id,joint,:] = \
						(-np.array(self.joints[id][joint].real)/1000.0).tolist() 
					self.rotation[id,joint,:] = \
						self.joints[id][joint].orientation.flatten()
				
				self.joints[id] = data.skeletons[skeleton][1:]
			else:
				# if not, we will add now id and save data in the joints list,
				# translation list and rotation list
				self.ids.append(data.skeletons[skeleton].user_id)
				id = len(self.ids)-1
				self.joints.append(data.skeletons[skeleton][1:])

				for joint in range(0,20):
					self.translation[id,joint,:] = \
						(-np.array(self.joints[id][joint].real)/1000.0).tolist() 
					self.rotation[id,joint,:] = \
						self.joints[id][joint].orientation.flatten()
		self.do()


	def do(self):
		"""
		Function to send all ids' TFs
		"""

		for id in self.ids:
			self.handle_tf(self.ids.index(id))


	def handle_tf(self,id):
		"""
		Function send Transform information of every point of skeletons
		Args:
			id(int): The id of detected person
		"""

		# Name every joint
		joint_names = ["Head","Neck","Torso","Waist","Left_Collar",
			"Left_Shoulder","Left_Elbow","Left_Wrist","Left_Hand",
			"Right_Collar","Right_Shoulder","Right_Elbow","Right_Wrist",
			"Right_Hand","Left_Hip","Left_Knee","Left_Ankle","Right_Hip",
			"Right_Knee","Right_Ankle"]
		# NOTE : why were there 20/25 joints
		joints_numbers = [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]

		# Calculate the rotation in Euler angles. [joint, xyz]
		euler_rotations = np.zeros([20,3])
		for (joint,n) in zip(joints_numbers,range(20)): 
			r = self.rotation[id,joint,:]
			m = np.mat([
				[r[0], r[1], r[2]],
				[r[3], r[4], r[5]],
				[r[6], r[7], r[8]]
			])
			euler_rotation = tf.transformations.euler_from_matrix(m,"rxyz")
			euler_rotations[n,:] = euler_rotation
		
		# send transform message
		for (joint, joint_name) in zip(joints_numbers, range(len(joint_names))):
			translation = self.translation[id,joint,:]
			rotation = tf.transformations.quaternion_from_euler(
				euler_rotations[joint_name, 0],
				euler_rotations[joint_name, 1],
				euler_rotations[joint_name, 2])
			time = rospy.Time.now()
			child = joint_names[joint_name]+"_"+str(id)
			parent = "/nuitrack_frame"

			self.transform_broadcaster.sendTransform(
				translation,
				rotation,
				time,
				child,
				parent)


if __name__ == '__main__':
	# Configure logging module
	logging.basicConfig(level=logging.DEBUG)

	SC = SkeletonConverter()
	SC.init_nuitrack()

	rospy.init_node("tf_skeletons",anonymous=True)

	SC.update()