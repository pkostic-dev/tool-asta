#!/usr/bin/env python3

import rospy
import tf
from PyNuitrack import py_nuitrack
import numpy as np
import logging


class SkeletonConverter:
	"""Converts Nuitrack skeleton data into TF data and publishes it.

	This class initialises a Nuitrack object. Converts the data obtained from it
	into TF skeleton data. And publishes the TF skeleton data to ROS with a
	TransformBroadcaster object.
	"""

	def __init__(self) -> None:
		"""Initialises other objects and creates class member variables

		Initialisation of class, creates Nuitrack and TransformBroadcaster
		objects. As well as all the class member variables.
		"""

		super().__init__()

		self.nuitrack = py_nuitrack.Nuitrack()

		# TransformBroadcaster initialisation
		logging.info("Initialising tf.TransformBroadcaster object.")
		self.transform_broadcaster = tf.TransformBroadcaster()

		# The ids of percieved skeletons
		self.ids = []
		# List containing joints of each skeleton
		# There is a total of 20 joints per skeleton
		self.joints = []
		# The translations of every joint in space
		# [ids, joints, xyz_coordinates]
		self.translation = np.zeros([10, 25, 3])
		# The rotation of every joint in space
		# [ids, joints, 3x3_matrices]
		self.rotation = np.zeros([10, 25, 9])


	def init_nuitrack(self) -> None:
		"""Initialises a Nuitrack object.
		
		Initialises a Nuitrack object. Searches for a Nuitrack compatible device
		and selects the first one. Checks the activation status of the license.
		Finally the Nuitrack object is run.
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


	def shutdown_hook(self) -> None:
		"""Shutdown hook for the Nuitrack object."""

		logging.info("Shutting down nuitrack.")
		if self.nuitrack != None:
			self.nuitrack.release()


	def update(self) -> None:
		"""Continuously update Nuitrack, get skeleton data, and load it."""

		while not rospy.is_shutdown():
			try:
				self.nuitrack.update()
				data = self.nuitrack.get_skeleton()
				self.store_skeletons(data)
				self.do()
			except KeyboardInterrupt:
				logging.info("Shutting down nuitrack.")
				if self.nuitrack != None:
					self.nuitrack.release()
				rospy.signal_shutdown("KeyboardInterrupt")
				raise SystemExit

	
	def store_skeletons(self, data) -> None:
		"""Load skeleton data heard from Nuitrack."""

		# load data of every id 
		for skeleton in range(0,len(data.skeletons)):
			# verify if this id was detected before
			if data.skeletons[skeleton].user_id in self.ids:
				# if id was detected before, we copy the joint's previous
				# translation and rotation to the translation list and rotation
				# list.
				# Then Overwrite the original data of joints with the new state 
				id = self.ids.index(data.skeletons[skeleton].user_id)

				self.store_joints(id)
				
				self.joints[id] = data.skeletons[skeleton][1:]
			else:
				# if not, we will add now id and save data in the joints list,
				# translation list and rotation list
				self.ids.append(data.skeletons[skeleton].user_id)
				id = len(self.ids)-1
				self.joints.append(data.skeletons[skeleton][1:])

				self.store_joints(id)


	def store_joints(self, id) -> None:
		for joint in range(0,20):
			# NOTE : find out why the array way negative ?
			self.translation[id,joint,:] = \
				(np.array(self.joints[id][joint].real)/1000.0).tolist() 
			self.rotation[id,joint,:] = \
				self.joints[id][joint].orientation.flatten()


	def do(self) -> None:
		"""Function to send all ids' TFs."""

		for id in self.ids:
			self.handle_tf(self.ids.index(id))


	def handle_tf(self, id) -> None:
		"""Function send Transform information of every point of skeletons"""

		# Calculate the rotation in Euler angles. [joint, xyz]
		euler_rotations = np.zeros([20, 3])

		for joint_number in range(20): 
			r = self.rotation[id, joint_number, :]
			m = np.mat([
				[r[0], r[1], r[2]],
				[r[3], r[4], r[5]],
				[r[6], r[7], r[8]]
			])
			euler_rotation = tf.transformations.euler_from_matrix(m, "rxyz")
			euler_rotations[joint_number, :] = euler_rotation
		
		# send transform message
		for joint_number in range(20):
			translation = self.translation[id,joint_number, :]
			rotation = tf.transformations.quaternion_from_euler(
				euler_rotations[joint_number, 0],
				euler_rotations[joint_number, 1],
				euler_rotations[joint_number, 2])
			time = rospy.Time.now()
			child = str(self.joints[id][joint_number].type)+"_"+str(id)
			parent = "/nuitrack_frame"

			self.transform_broadcaster.sendTransform(
				translation,
				rotation,
				time,
				child,
				parent)


	def launch(self) -> None:
		"""Launches the nuitrack, ros node and starts updating."""

		self.init_nuitrack()
		rospy.init_node("tf_skeletons",anonymous=True)
		self.update()


if __name__ == '__main__':
	# Configure logging module
	logging.basicConfig(level=logging.DEBUG)

	SC = SkeletonConverter()
	SC.launch()