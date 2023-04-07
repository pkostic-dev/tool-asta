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

		# The ids of perced objects
		self.id = []
		# The joints list of skeleton. The length will be 25. Because Nuitrack
		# describe skeleton with 25 joints. Every joint item has information of
		# translation and rotation
		self.joints = []
		# The translations of every joint in space
		self.translation = np.zeros([10,25,3])
		# The rotation of every joint in space
		self.rotation = np.zeros([10,25,9])


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


	def convert(self, data):
		"""Converts nuitrack skeleton data to qt skeleton data format"""
		
		skeletons = []
		for skelly in data.skeletons:
			id = skelly.user_id
			joints = skelly[1:]
			new_skelly = {
				"id" : id,
				"joints" : joints	
			}
			skeletons.append(new_skelly)
		data = {
			"skeletons" : skeletons
		}
		return data


	def update(self):
		"""Continuously update Nuitrack, get, convert, and load skeleton data."""

		while not rospy.is_shutdown():
			self.nuitrack.update()
			data = self.convert(self.nuitrack.get_skeleton())
			data = self.nuitrack.get_skeleton()
			print(data)
			# NOTE : From this point forward, black box.
			self.load(data)

	
	def load(self,data):
		"""
		Function for load data heard from nuitrack	

		Args:
		data: The data heard from nuitrack
		"""

		# load data of every id 
		for n in range(0,len(data.skeletons)):
			# verifie if this id was detected before
			
			if data.skeletons[n].user_id in self.id:
				# if id was detected before, we copy the joint's previous
				# translation and rotation to the translation list and rotation
				# list.
				# Then Overwrite the original data of joints with the new state 
				i = self.id.index(data.skeletons[n].user_id)

				for j in range(0,20):
					self.translation[i,j,:] = \
						(-np.array(self.joints[i][j].real)/1000.0).tolist()
					self.rotation[i,j,:] = self.joints[i][j].orientation.flatten()
				self.joints[i] = data.skeletons[n][1:]
			else:
				# if not, we will add now id and save data in the joints list,
				# translation list and rotation list
				self.id.append(data.skeletons[n].user_id)
				i = len(self.id)-1
				self.joints.append(data.skeletons[n][1:])
				for j in range(0,20):
					self.translation[i,j,:] = \
						(-np.array(self.joints[i][j].real)/1000.0).tolist()
					self.rotation[i,j,:] = self.joints[i][j].orientation.flatten()
		self.do()


	def do(self):
		"""
		Function to send all ids' TFs
		"""

		for i in self.id:
			self.handle_tf(self.id.index(i))


	def handle_tf(self,i):
		"""
		Function send Transform information of every point of skeletons
		Args:
			i(int): The id of detected person
		"""

		# TODO : check if joint list and number list correct
		# name every joint
		joint_name = ["Head","Neck","Torso","Waist","Left_Collar",
			"Left_Shoulder","Left_Elbow","Left_Wrist","Left_Hand",
			"Right_Collar","Right_Shoulder","Right_Elbow","Right_Wrist",
			"Right_Hand","Left_Hip","Left_Knee","Left_Ankle","Right_Hip",
			"Right_Knee","Right_Ankle"]
		# there a 3 joints don't have data, so we use just 23 joints here.
		joints_number = [1,2,3,4,5,6,7,8,9,11,12,13,14,15,17,18,19,21,22,23]

		# calculate the rotation in Euler angles.
		rot_euler = np.zeros([20,3])
		for (j,n) in zip(joints_number,range(20)): 
			r = self.rotation[i,j,:]
			m = np.mat([[r[0],r[1],r[2]],[r[3],r[4],r[5]],[r[6],r[7],r[8]]])
			rot = tf.transformations.euler_from_matrix(m,"rxyz")
			rot_euler[n,:] = rot
		
		# send transform message
		for (k,j) in zip(joints_number, range(len(joint_name))):
			translation = self.translation[i,k,:]
			rotation = tf.transformations.quaternion_from_euler(
				rot_euler[j,0],
				rot_euler[j,1],
				rot_euler[j,2])
			time = rospy.Time.now()
			child = joint_name[j]+"_"+str(i)
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