#!/usr/bin/env python

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap

import tf
from tf import TransformListener
from tf import TransformBroadcaster
from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix
from random import gauss

import math
import time

import numpy as np
from numpy.random import random_sample
from sklearn.neighbors import NearestNeighbors

class TransformHelpers:
	""" Some convenience functions for translating between various representions of a robot pose.
		TODO: nothing... you should not have to modify these """

	@staticmethod
	def test():
		""" Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
		print "test"

	@staticmethod
	def convert_translation_rotation_to_pose(translation, rotation):
		""" Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
		return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

	@staticmethod
	def convert_pose_inverse_transform(pose):
		""" Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) """
		translation = np.zeros((4,1))
		translation[0] = -pose.position.x
		translation[1] = -pose.position.y
		translation[2] = -pose.position.z
		translation[3] = 1.0

		rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		euler_angle = euler_from_quaternion(rotation)
		rotation = np.transpose(rotation_matrix(euler_angle[2], [0,0,1]))		# the angle is a yaw
		transformed_translation = rotation.dot(translation)

		translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
		rotation = quaternion_from_matrix(rotation)
		return (translation, rotation)

	@staticmethod
	def convert_pose_to_xy_and_theta(pose):
		""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
		orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
		angles = euler_from_quaternion(orientation_tuple)
		return (pose.position.x, pose.position.y, angles[2])
