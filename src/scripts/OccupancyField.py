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


""" Difficulty Level 2 """
class OccupancyField:
	""" Stores an occupancy field for an input map.  An occupancy field returns the distance to the closest
		obstacle for any coordinate in the map
		Attributes:
			map: the map to localize against (nav_msgs/OccupancyGrid)
			closest_occ: the distance for each entry in the OccupancyGrid to the closest obstacle
	"""

	def __init__(self, map = None):
		self.map = map		# save this for later
		# TODO: implement this (level 2)

	def get_closest_obstacle_distance(self,x,y):
		""" Compute the closest obstacle to the specified (x,y) coordinate in the map.  If the (x,y) coordinate
			is out of the map boundaries, nan will be returned. """
		# TODO: implement this 
