#!/usr/bin/env python

import rospy

from std_msgs.msg import Header, String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import Twist, Vector3

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
from copy import deepcopy


from TransformHelpers import TransformHelpers
from Particle import Particle
from OccupancyField import OccupancyField

class ParticleFilter:
    """ The class that represents a Particle Filter ROS Node
        Attributes list:
            initialized: a Boolean flag to communicate to other class methods that initializaiton is complete
            base_frame: the name of the robot base coordinate frame (should be "base_link" for most robots)
            map_frame: the name of the map coordinate frame (should be "map" in most cases)
            odom_frame: the name of the odometry coordinate frame (should be "odom" in most cases)
            scan_topic: the name of the scan topic to listen to (should be "scan" in most cases)
            n_particles: the number of particles in the filter
            d_thresh: the amount of linear movement before triggering a filter update
            a_thresh: the amount of angular movement before triggering a filter update
            laser_max_distance: the maximum distance to an obstacle we should use in a likelihood calculation
            pose_listener: a subscriber that listens for new approximate pose estimates (i.e. generated through the rviz GUI)
            particle_pub: a publisher for the particle cloud
            laser_subscriber: listens for new scan data on topic self.scan_topic
            tf_listener: listener for coordinate transforms
            tf_broadcaster: broadcaster for coordinate transforms
            particle_cloud: a list of particles representing a probability distribution over robot poses
            current_odom_xy_theta: the pose of the robot in the odometry frame when the last filter update was performed.
                                   The pose is expressed as a list [x,y,theta] (where theta is the yaw)
            map: the map we will be localizing ourselves in.  The map should be of type nav_msgs/OccupancyGrid
    """
    def __init__(self, test = False):
        
        self.test = test
        if self.test:
            print "Running particle filter in test mode"
        else:
            print "Running particle filter"
        self.initialized = False        # make sure we don't perform updates before everything is setup
        rospy.init_node('pf')           # tell roscore that we are creating a new node named "pf"

        self.base_frame = "base_link"   # the frame of the robot base
        self.map_frame = "map"          # the name of the map coordinate frame
        self.odom_frame = "odom"        # the name of the odometry coordinate frame
        self.scan_topic = "scan"        # the topic where we will get laser scans from 

        self.n_particles = 20          # the number of particles to use

        self.d_thresh = 0.1             # the amount of linear movement before performing an update
        self.a_thresh = math.pi/6       # the amount of angular movement before performing an update

        self.laser_max_distance = 2.0   # maximum penalty to assess in the likelihood field model
        self.scan_count = 0 

        self.robot_pose =  Pose(position=Point(x=.38, y=.6096,z=0), orientation=Quaternion(x=0, y=0, z=0, w=1))        

        # Setup pubs and subs

        # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        self.pose_listener = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.update_initial_pose)
        # publish the current particle cloud.  This enables viewing particles in rviz.
        self.particle_pub = rospy.Publisher("particlecloud", PoseArray)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # laser_subscriber listens for data from the lidar
        self.laser_subscriber = rospy.Subscriber(self.scan_topic, LaserScan, self.scan_received)

        # enable listening for and broadcasting coordinate transforms
        self.tf_listener = TransformListener()
        self.tf_broadcaster = TransformBroadcaster()

        self.particle_cloud = []
        self.ang_spread = 10.0/180.0 * math.pi
        self.lin_spread = .1
        self.current_odom_xy_theta = []

        #Set up constants for Guassian Probability
        self.variance = .1
        self.gauss_constant = math.sqrt(2*math.pi)
        self.expected_value = 0 

        # request the map from the map server, the map should be of type nav_msgs/OccupancyGrid
        # TODO: fill in the appropriate service call here.  The resultant map should be assigned be passed
        #       into the init method for OccupancyField

        # for now we have commented out the occupancy field initialization until you can successfully fetch the map
        #self.occupancy_field = OccupancyField(map)

        # request the map from the map server, the map should be of type nav_msgs/OccupancyGrid
        # TODO: fill in the appropriate service call here.  The resultant map should be assigned be passed

        #       into the init method for OccupancyField

        # We make a fake approximate map of an empty 2 by 2 square for testing to keep it simple.
        # If this worked better, we'd expand, but we never got it to work better.
        map = OccupancyGrid()
        map.header.frame_id = '/odom'
        map.info.map_load_time = rospy.Time.now()
        map.info.resolution = .1 # The map resolution [m/cell]
        map.info.width = 288
        map.info.height = 288
        
        map.data = [[0 for _ in range(map.info.height)] for _ in range(map.info.width)] 
        
        for row in range(map.info.height):
            map.data[0][row] = 1
            map.data[map.info.width-1][row] = 1
        for col in range(map.info.width):
            map.data[col][0] = 1
            map.data[col][map.info.height -1] = 1

        self.occupancy_field = OccupancyField(map)
        if self.test:
            print "Initialized"
        self.initialized = True

    def update_robot_pose(self):
        """ Update the estimate of the robot's pose given the updated particles.
            There are two logical methods for this:
                (1): compute the mean pose (level 2)
                (2): compute the most likely pose (i.e. the mode of the distribution) (level 1)
        """
        if self.test:
            print "updating robot's pose"
        

        # first make sure that the particle weights are normalized
        self.normalize_particles()
        total_x = 0.0
        total_y = 0.0
        total_theta = 0.0

	#calculates mean position of particle cloud according to particle weight
	#particles are normalized so sum of multiples will return mean
        for particle in self.particle_cloud: 
            total_x += particle.x * particle.w
            total_y += particle.y * particle.w
            total_theta += particle.theta * particle.w
        total_theta = math.cos(total_theta/2)

        #set the robot pose to new position
        self.robot_pose =  Pose(position=Point(x= +total_x, y=total_y,z=0), orientation=Quaternion(x=0, y=0, z=0, w=total_theta))

        if self.test:
            print "updated pose:"
            print self.robot_pose

    def update_particles_with_odom(self, msg):
        """ Implement a simple version of this (Level 1) or a more complex one (Level 2) """

        if self.test:
            print "Updating particles from odom"
        
        new_odom_xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # compute the change in x,y,theta since our last update
        if self.current_odom_xy_theta:
            old_odom_xy_theta = self.current_odom_xy_theta
            delta = (new_odom_xy_theta[0] - self.current_odom_xy_theta[0], new_odom_xy_theta[1] - self.current_odom_xy_theta[1], new_odom_xy_theta[2] - self.current_odom_xy_theta[2])
            self.current_odom_xy_theta = new_odom_xy_theta
        else:
            self.current_odom_xy_theta = new_odom_xy_theta
            return

	#function moves the particle cloud according to the odometry data
	#particle noise is added using gaussian distribution
	#standard deviation of gaussian dist was experimentally measured

        x_sd = .001
        y_sd = .001
        theta_sd = .1

        for particle in self.particle_cloud:
            particle.x += np.random.normal(particle.x + delta[0], x_sd)
            particle.y += np.random.normal(particle.y + delta[1], y_sd)
            particle.theta += np.random.normal(particle.theta + delta[2], theta_sd)


    def map_calc_range(self,x,y,theta):
        """ Difficulty Level 3: implement a ray tracing likelihood model... Let me know if you are interested """
        # TODO: nothing unless you want to try this alternate likelihood model
        pass

    def resample_particles(self):
        """ Resample the particles according to the new particle weights """
        # make sure the distribution is normalized
        if self.test:
            print "Resampling Particles"
        
        # TODO: fill out the rest of the implementation

	#draw a random sample of particles from particle cloud
	#then normalize the remaining particles
        weights = [particle.w for particle in self.particle_cloud]
        self.particle_cloud = self.draw_random_sample(
            self.particle_cloud, weights, self.n_particles) 
        self.normalize_particles()

        print "SAMPLING"
        print len(self.particle_cloud)
        print self.particle_cloud 
        

    def update_particles_with_laser(self, msg):
        """ Updates the particle weights in response to the scan contained in the msg """
        if self.test:
            print "Updating particles from laser scan"
        
        
        for particle in self.particle_cloud:
            closest_particle_object_distance, closest_particle_object_theta = self.occupancy_field.get_closest_obstacle_distance(particle.x, particle.y) 
            
            closest_actual_object_distance = 1000 
            for i in range(1, 360):
                if msg.ranges[i] > 0.0 and msg.ranges[i] < closest_actual_object_distance:
                    closest_actual_object_distance = msg.ranges[i]
                    closest_actual_object_theta = (i/360.0)*2*math.pi
            

            print closest_particle_object_distance-closest_actual_object_distance
            print "actual theta"
            print closest_actual_object_theta
            print "part theta"
            print closest_particle_object_theta

            if self.gauss_particle_probability(closest_particle_object_distance-closest_actual_object_distance) > 0:
                print particle.x,particle.y
            particle.w = self.gauss_particle_probability(closest_particle_object_distance-closest_actual_object_distance)
            
            particle.theta =  closest_actual_object_theta - closest_particle_object_theta
            print particle.w
            # particle.unnormalized_w = (g_probability+particle.unnormalized_w)/2
        # raw_input()
    def gauss_particle_probability(self, difference):
        return (1/(self.variance*self.gauss_constant))*math.exp(-.5*((difference - self.expected_value)/self.variance)**2)
   

    @staticmethod
    def angle_normalize(z):
        """ convenience function to map an angle to the range [-pi,pi] """
        return math.atan2(math.sin(z), math.cos(z))

    @staticmethod
    def angle_diff(a, b):
        """ Calculates the difference between angle a and angle b (both should be in radians)
            the difference is always based on the closest rotation from angle a to angle b
            examples:
                angle_diff(.1,.2) -> -.1
                angle_diff(.1, 2*math.pi - .1) -> .2
                angle_diff(.1, .2+2*math.pi) -> -.1
        """
        a = ParticleFilter.angle_normalize(a)
        b = ParticleFilter.angle_normalize(b)
        d1 = a-b
        d2 = 2*math.pi - math.fabs(d1)
        if d1 > 0:
            d2 *= -1.0
        if math.fabs(d1) < math.fabs(d2):
            return d1
        else:
            return d2

    @staticmethod
    def weighted_values(values, probabilities, size):
        """ Return a random sample of size elements form the set values with the specified probabilities
            values: the values to sample from (numpy.ndarray)
            probabilities: the probability of selecting each element in values (numpy.ndarray)
            size: the number of samples
        """
        bins = np.add.accumulate(probabilities)
        return values[np.digitize(random_sample(size), bins)]

    @staticmethod
    def draw_random_sample(choices, probabilities, n):
        """ Return a random sample of n elements from the set choices with the specified probabilities
          choices: the values to sample from represented as a list
          probabilities: the probability of selecting each index represented as a list
          n: the number of samples
        """
        values = np.array(range(len(choices)))
        probs = np.array(probabilities)
        bins = np.add.accumulate(probs)
        inds = values[np.digitize(random_sample(n), bins)]
        samples = [deepcopy(choices[ind]) for ind in inds]
        return samples


    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter based on a pose estimate.
            These pose estimates could be generated by another ROS Node or could come from the rviz GUI """
        if self.test:
            print "Updating Initial Pose"
        
        xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(msg.pose.pose)
        self.initialize_particle_cloud(xy_theta)
        self.fix_map_to_odom_transform(msg)


    def initialize_particle_cloud(self, xy_theta=None):
        """ Initialize the particle cloud.
            Arguments
            xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
                      particle cloud around.  If this input is ommitted, the odometry will be used """
        if self.test:
            print "Initializing Cloud"
            
	if xy_theta == None:
            xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(self.robot_pose)
            print self.robot_pose
            print xy_theta
            # raw_input()
        self.particle_cloud = []
        # TODO create particles

	#create a normal distribution of particles around starting position
	#then normalize and update pose accordingly
        x_vals = np.random.normal(xy_theta[0], self.lin_spread, self.n_particles)
        y_vals = np.random.normal(xy_theta[1], self.lin_spread, self.n_particles)
        t_vals = np.random.normal(xy_theta[2], self.ang_spread, self.n_particles)
        self.particle_cloud = [Particle(x_vals[i], y_vals[i], t_vals[i], 1)
                               for i in xrange(self.n_particles)]
        
        self.normalize_particles()
        self.update_robot_pose()
        # TODO(mary): create particles
        

    def normalize_particles(self):
        """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
        # TODO(mary): implement this
        total_weight = sum([particle.w for particle in self.particle_cloud]) * 1.0
        print "average weight"
        print total_weight/self.n_particles

        for particle in self.particle_cloud:
          particle.w /= total_weight 
      
    def publish_particles(self, msg):
        particles_conv = []
        for p in self.particle_cloud:
            particles_conv.append(p.as_pose())
        # actually send the message so that we can view it in rviz
        self.particle_pub.publish(PoseArray(header=Header(stamp=rospy.Time.now(),frame_id=self.map_frame),poses=particles_conv))

    def scan_received(self, msg):
        self.scan_count +=1
        """ This is the default logic for what to do when processing scan data.  Feel free to modify this, however,
            I hope it will provide a good guide.  The input msg is an object of type sensor_msgs/LaserScan """
        if not(self.initialized):
            # wait for initialization to complete
            return

        if not(self.tf_listener.canTransform(self.base_frame,msg.header.frame_id,msg.header.stamp)):
            # need to know how to transform the laser to the base frame
            # this will be given by either Gazebo or neato_node
            return

        if not(self.tf_listener.canTransform(self.base_frame,self.odom_frame,msg.header.stamp)):
            # need to know how to transform between base and odometric frames
            # this will eventually be published by either Gazebo or neato_node
            return

        # calculate pose of laser relative ot the robot base
        p = PoseStamped(header=Header(stamp=rospy.Time(0),frame_id=msg.header.frame_id))
        self.laser_pose = self.tf_listener.transformPose(self.base_frame,p)

        # find out where the robot thinks it is based on its odometry
        p = PoseStamped(header=Header(stamp=msg.header.stamp,frame_id=self.base_frame), pose=Pose())
        self.odom_pose = self.tf_listener.transformPose(self.odom_frame, p)
        # store the the odometry pose in a more convenient format (x,y,theta)
        # print "TEST"
        new_odom_xy_theta = TransformHelpers.convert_pose_to_xy_and_theta(self.odom_pose.pose)
        # print new_odom_xy_theta
        # self.odom_pose.pose.position.x +=1
        # new_odom_xy_theta = TransformHelpers.convert_pose_to_xy_and_theta( self.robot_pose)
        # print new_odom_xy_theta
        # print "DONE"
        

        if not(self.particle_cloud):
            # now that we have all of the necessary transforms we can update the particle cloud
            self.initialize_particle_cloud()
            # cache the last odometric pose so we can only update our particle filter if we move more than self.d_thresh or self.a_thresh
            self.current_odom_xy_theta = new_odom_xy_theta
            # update our map to odom transform now that the particles are initialized
            self.fix_map_to_odom_transform(msg)
                    # if self.test or self.scan_count % 1 is 0:
            print "updated pose:"
            print self.robot_pose
        elif (math.fabs(new_odom_xy_theta[0] - self.current_odom_xy_theta[0]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[1] - self.current_odom_xy_theta[1]) > self.d_thresh or
              math.fabs(new_odom_xy_theta[2] - self.current_odom_xy_theta[2]) > self.a_thresh):
            
            # we have moved far enough to do an update!
            self.update_particles_with_odom(msg)    # update based on odometry
            self.update_particles_with_laser(msg)   # update based on laser scan
            self.update_robot_pose()                # update robot's pose
            self.resample_particles()               # resample particles to focus on areas of high density
            self.fix_map_to_odom_transform(msg)     # update map to odom transform now that we have new particles
            # print self.particles
        # publish particles (so things like rviz can see them)
        # print new_odom_xy_theta
        # print new_odom_xy_theta[0] - self.current_odom_xy_theta[0]
        # print new_odom_xy_theta[1] - self.current_odom_xy_theta[1]
        # print new_odom_xy_theta[2] - self.current_odom_xy_theta[2]
        # print "pose"
        # print self.robot_pose
        # # print "publish"
        # if self.test or self.scan_count % 1 is 0:
        #     print "updated pose:"
        #     print self.robot_pose
        self.publish_particles(msg)

    def fix_map_to_odom_transform(self, msg):
        """ Super tricky code to properly update map to odom transform... do not modify this... Difficulty level infinity. """
        (translation, rotation) = TransformHelpers.convert_pose_inverse_transform(self.robot_pose)
        p = PoseStamped(pose=TransformHelpers.convert_translation_rotation_to_pose(translation,rotation),header=Header(stamp=msg.header.stamp,frame_id=self.base_frame))
        self.odom_to_map = self.tf_listener.transformPose(self.odom_frame, p)
        (self.translation, self.rotation) = TransformHelpers.convert_pose_inverse_transform(self.odom_to_map.pose)

    def broadcast_last_transform(self):
        """ Make sure that we are always broadcasting the last map to odom transformation.
            This is necessary so things like move_base can work properly. """
        if not(hasattr(self,'translation') and hasattr(self,'rotation')):
            return
        self.tf_broadcaster.sendTransform(self.translation, self.rotation, rospy.get_rostime(), self.odom_frame, self.map_frame)

if __name__ == '__main__':
    n = ParticleFilter(True)
    r = rospy.Rate(5)
 
    while not(rospy.is_shutdown()):
        # in the main loop all we do is continuously broadcast the latest map to odom transform
        n.broadcast_last_transform()
-       r.sleep()
