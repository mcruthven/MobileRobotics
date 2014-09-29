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


class Particle:
	""" Represents a hypothesis (particle) of the robot's pose consisting of x,y and theta (yaw)
		Attributes:
			x: the x-coordinate of the hypothesis relative to the map frame
			y: the y-coordinate of the hypothesis relative ot the map frame
			theta: the yaw of the hypothesis relative to the map frame
			w: the particle weight (the class does not ensure that particle weights are normalized
	"""

	def __init__(self,x=0.0,y=0.0,theta=0.0,w=1.0):
		""" Construct a new Particle
			x: the x-coordinate of the hypothesis relative to the map frame
			y: the y-coordinate of the hypothesis relative ot the map frame
			theta: the yaw of the hypothesis relative to the map frame
			w: the particle weight (the class does not ensure that particle weights are normalized """ 
		self.w = w
		self.theta = theta
		self.x = x
		self.y = y

	def as_pose(self):
		""" A helper function to convert a particle to a geometry_msgs/Pose message """
		orientation_tuple = tf.transformations.quaternion_from_euler(0,0,self.theta)
		return Pose(position=Point(x=self.x,y=self.y,z=0), orientation=Quaternion(x=orientation_tuple[0], y=orientation_tuple[1], z=orientation_tuple[2], w=orientation_tuple[3]))

	# TODO: define additional helper functions if needed

