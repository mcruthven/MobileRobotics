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

    def __init__(self, map, test = False ):
        self.scale = .025
        self.map = map  
        self.map_data = map.data        # save this for later
        self.max_distance = max(self.map.info.height, self.map.info.width)
        self.proximity_grid = []
        self.initialize_proximity_grid()
        self.test = test


    def initialize_proximity_grid(self):
        self.proximity_grid = [[-1 for _ in range(self.map.info.height)] for _ in range(self.map.info.width)] 
        
        # for x in range(self.map.info.width):
        #     self.proximity_grid.append([])
        # for x in range(self.map.info.width):
        #     print x
        #     for y in range(self.map.info.height):
        #         dist = int(self.calc_closest_obstacle_distance(x, y))
        #         self.proximity_grid[x][y] =dist

        #         if dist > self.max_distance:
        #             self.max_distance = dist
        #     print self.proximity_grid[x]

       
    def calc_closest_obstacle_distance(self, x, y):
        if self.map_data[x][y] is 1:
            return 0
        distance = 1    
        object_found = False
        while not object_found:
            if (self.map_data[x+distance][y] is 1) or (self.map_data[x-distance][y] is 1) or (self.map_data[x][y + distance] is 1) or (self.map_data[x][y - distance] is 1):
                object_found = True
            else:
                distance +=1
        return self.calc_obstacle_within_limit(x, y, distance)
                  

    def calc_obstacle_within_limit (self, x, y, distance):
        objects = []
        closet_point = distance
        xmin, ymin, xmax, ymax = self.check_field_edges(x, y, distance)
        for xdel in range(xmin, xmax):
            if abs(xdel) < closet_point: 
                for ydel in range(ymin, ymax):
                    if self.map_data[x + xdel][y + ydel] is 1:
                        objects.append({"x": x + xdel, "y": y + ydel} )
                        # if self.test:   
                        # print "x" + str(x) + " y" + str(y)
                        # print xdel, ydel
                        # print ((xdel*xdel) + (ydel*ydel))
                        pt_dist = math.sqrt((xdel*xdel + ydel*ydel))
                        if pt_dist < closet_point:
                            closet_point = pt_dist

        return closet_point


    def check_field_edges (self, x, y, distance):
        xmin = -1*distance
        ymin = -1*distance
        xmax = distance
        ymax = distance 
        if x + xmin < 0:
            xmin = -1*x
        if y + ymin < 0:
            ymin = -1*y
        if x + xmax > self.map.info.width:
            xmax = self.map.info.width - x # minus 1 for bc 0 based
        if y + ymax > self.map.info.height:
            ymax = self.map.info.height - y # minus 1 for bc 0 based
        return xmin, ymin, xmax, ymax

    def get_closest_obstacle_distance(self,x,y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in the map.  If the (x,y) coordinate
            is out of the map boundaries, nan will be returned. """
        x = int(round(x/self.scale))
        y = int(round(y/self.scale))
        print x, y
        if self.proximity_grid[x][y] is -1:
            self.proximity_grid[x][y] = self.calc_closest_obstacle_distance(x, y)
        return self.proximity_grid[x][y]