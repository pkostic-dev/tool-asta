#!/usr/bin/env python3
import rospy
import tf
from PyNuitrack import py_nuitrack
import numpy as np
import logging

#import cv2
#from itertools import cycle

logging.basicConfig(level=logging.DEBUG)

class NuitrackSkeletonListener:
	"""Listens for Nuitrack skeleton data.

	This class initialises a Nuitrack object, setups the first device found, and
	starts listening for skeleton data.
	"""
	def __init__(self):
		super().__init__()

		# Nuitrack initialisation
		self.nuitrack = self.init_nuitrack()

	def init_nuitrack(self) -> py_nuitrack.Nuitrack:
		"""Initialises a Nuitrack object.
		
		This method initialises a Nuitrack object. It searches for a Nuitrack
		compatible device and selects the first one. It checks the activation
		status of the license. Finally it runs the Nuitrack object.
		"""
		logging.info("Initialising py_nuitrack.Nuitrack object.")
		nuitrack = py_nuitrack.Nuitrack()
		nuitrack.init()

		logging.info("Getting Nuitrack device list...")
		devices = nuitrack.get_device_list()
		for i, dev in enumerate(devices):
			logging.info("Device %d:\
				\n\tname = %s,\
				\n\tserial_number = %s",
				i,
				dev.get_name(),
				dev.get_serial_number())
			if i == 0:
				if dev.get_activation() == "None":
					logging.warn("No active license for this device.")
					nuitrack.release()
					return
				
				# If a license hasn't been activated with the Nuitrack Runtime
				# package it can be activated using Python API
				# dev.activate("ACTIVATION_KEY")

				logging.info("Device %d activation status : %s",
		 			i,
					dev.get_activation())
				logging.info("Setting device %d as active.", i)
				nuitrack.set_device(dev)

		logging.info("Creating nuitrack modules.")
		nuitrack.create_modules()
		logging.info("Running nuitrack.")
		nuitrack.run()

		return nuitrack

class TFSkeletonPublisher:
	"""Publishes tf skeleton data.

	This class initialises a TransformBroadcaster object and publishes tf
	data skeleton with it.
	"""
	def __init__(self):
		super().__init__()

		# Transform Broadcaster initialisation
		logging.info("Initialising tf.TransformBroadcaster object.")
		self.transform_broadcaster = tf.TransformBroadcaster()

	def publish_data(self, skeleton_data):
		pass

	
def nuitrack_data_to_qt_msg(data):
	skeletons = []
	for skelly in data.skeletons:
		id = skelly.user_id
		joints = skelly[1:]
		new_skelly = dict(id = id, joints = joints)
		skeletons.append(new_skelly)
	data = dict(skeletons = skeletons)
	return data

class SkeletonConverter:
	"""Converts nuitrack skeleton data into tf data.

	...
	"""

	def __init__(self):
		"""
		Initialisation of class, creates Nuitrack and TransformBroadcaster
		objects.
		Arg:
			self: The object pointer
		"""

		super().__init__()

		

		# TODO : update names
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
		

		self.timestamp = 0
		self.skeletons_num = 0
		self.skeletons = []

	

	def shutdown_hook(self):
		logging.info("Shutting down nuitrack.")
		self.nuitrack.release()
	
	def load_info(self,data):
		"""
		Function for load data heard from nuitrack	

		Args:
		self: The object pointer
		data: The data heard from nuitrack
		"""
		# load data of every id 
		for n in range(0,len(data.skeletons)):
			# verifie if this id was detected before
			if data.skeletons[n].id in self.id:
				# if id was detected before, we copy the joint's previous
				# translation and rotation to the translation list and rotation
				# list.
				# Then Overwrite the original data of joints with the new state 
				i = self.id.index(data.skeletons[n].id)
				for j in range(0,25):
					self.translation[i,j,:] = \
						(-np.array(self.joints[i][j].real)/1000.0).tolist()
					self.rotation[i,j,:] = self.joints[i][j].orientation
				self.joints[i] = data.skeletons[n].joints
			else:
				# if not, we will add now id and save data in the joints list,
				# translation list and rotation list
				self.id.append(data.skeletons[n].id)
				i = len(self.id)-1
				self.joints.append(data.skeletons[n].joints)
				for j in range(0,25):
					self.translation[i,j,:] = \
						(-np.array(self.joints[i][j].real)/1000.0).tolist()
					self.rotation[i,j,:] = self.joints[i][j].orientation
		self.do()


	def msg_listener(self):
		""" 
		subscibe nuitrack msg
		"""
		rospy.on_shutdown(self.shutdown_hook)

		while not rospy.is_shutdown():
			self.nuitrack.update()
			data = nuitrack_data_to_qt_msg(self.nuitrack.get_skeleton())
			self.load_info(data)

	
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
				rot_euler[j,0],rot_euler[j,1],rot_euler[j,2])
			time = rospy.Time.now()
			child = joint_name[j]+"_"+str(i)
			parent = "/nuitrack_frame"
			self.br.sendTransform(translation, rotation, time, child, parent)


if __name__ == '__main__':
	NSL = NuitrackSkeletonListener()

	logging.info("Releasing nuitrack.")
	NSL.nuitrack.release()
	#TF=SkeletonConverter()
	#TF.msg_listener()

# 1. init nuitrack, get device, run
# 2. init vars
# 3. get data from nuitrack
# 4. translate data into tf
# 5. publish data