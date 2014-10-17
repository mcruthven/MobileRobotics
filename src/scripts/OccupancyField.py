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
        self.scale = .00625
        self.map = map  
        self.map_data = map.data        # save this for later
        self.max_distance = max(self.map.info.height, self.map.info.width)
        self.proximity_grid = []
        self.initialize_proximity_grid()
        self.test = test


    def initialize_proximity_grid(self):
        self.proximity_grid = [[-1 for _ in range(self.map.info.height)] for _ in range(self.map.info.width)] 
        
       
    def calc_closest_obstacle_distance(self, x, y):
        """" Calculates the distance to the closest object """
        #if the point is an object, return 0
        if self.map_data[x][y] is 1:
            return {"distance": 0, "theta": 0,}
        distance = 1    
        object_found = False
        closet_point = {}
        #find the closest object straight up, down, left or right since they're very quick to compute and can lower or search radius
        while not object_found:
            if (self.map_data[x+distance][y] is 1): 
                closet_point["theta"] = 0
                object_found = True
            elif (self.map_data[x-distance][y] is 1):
                object_found = True
                closet_point["theta"] = math.pi
            elif (self.map_data[x][y + distance] is 1):
                object_found = True
                closet_point["theta"] = .5*math.pi
            elif (self.map_data[x][y - distance] is 1):
                object_found = True
                closet_point["theta"] = 1.5* math.pi
            else:
                distance +=1
        closet_point["distance"] = distance
        
        return self.calc_obstacle_within_limit(x, y, closet_point)
                  

    def calc_obstacle_within_limit (self, x, y, closet_point):
        """ Given a point and a guaranteed distance to an object, searches around the point for objects within a square of side length 2*distance  """
        objects = []
        
        #get limits of x and y
        xmin, ymin, xmax, ymax = self.check_field_edges(x, y, closet_point["distance"])

        #for each delta x and each delta y, check for object
        #store the closest object until we find another object then update
        for xdel in range(xmin, xmax):
            if abs(xdel) < closet_point["distance"]: 
                for ydel in range(ymin, ymax):
                    if self.map_data[x + xdel][y + ydel] is 1:
                        objects.append({"x": x + xdel, "y": y + ydel} )
                        pt_dist = math.sqrt((xdel*xdel + ydel*ydel))
                        if pt_dist < closet_point:
                            closet_point["distance"] = pt_dist
                            closet_point["theta"]= math.atan(float(ydel)/max(float(xdel),.000001))
        return closet_point


    def check_field_edges (self, x, y, distance):
        """ Takes in an x, y and starting distance and checks how close the point is to the edges of the room.
            Returns the max and min we can add to x and y and stay within the room. """
        xmin = -1*distance
        ymin = -1*distance
        xmax = distance
        ymax = distance 

        if x + xmin < 0:
            xmin = -1*x
        if y + ymin < 0:
            ymin = -1*y
        if x + xmax > self.map.info.width:
            xmax = self.map.info.width - x 
        if y + ymax > self.map.info.height:
            ymax = self.map.info.height - y
        return xmin, ymin, xmax, ymax

    def get_closest_obstacle_distance(self,x,y):
        """ Compute the closest obstacle to the specified (x,y) coordinate in the map.  If the (x,y) coordinate
            is out of the map boundaries, nan will be returned. """
        x = int(round(x/self.scale))
        y = int(round(y/self.scale))
        # if we haven't already calculated for this x,y calculate and store the dist and theta
        if self.proximity_grid[x][y] is -1:
            self.proximity_grid[x][y] = self.calc_closest_obstacle_distance(x, y)
        return (self.proximity_grid[x][y]["distance"]*self.scale, self.proximity_grid[x][y]["theta"])