import numpy as np
from copy import deepcopy
import random
from numpy.random import random_sample
from OccupancyField import OccupancyField
import math
import random

from nav_msgs.msg import OccupancyGrid

import matplotlib.pyplot as plt

class Particle(object):
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

  def __repr__(self):
    return repr((self.x, self.w))

class ParticleFilter(object):

  def __init__(self, pt=None):
    self.n_particles = 10
    self.lin_spread = .05
    self.ang_spread = 1
    self.pt = pt if pt else (5,5,5)

    self.variance = 10
    self.gauss_constant = math.sqrt(2*math.pi)
    self.expected_value = 0 

    map = OccupancyGrid()
    map.header.frame_id = '/odom'
    # map.info.map_load_time = rospy.Time.now()
    map.info.resolution = .1 # The map resolution [m/cell]
    map.info.width = 1000
    map.info.height = 1000

    map.data = [[0 for _ in range(map.info.height)] for _ in range(map.info.width)]

    # for row in range(map.info.height):
    #     for col in range(map.info.width):
    #         print str(col) + str(row)

    #         map.data[col] = 0
    map.data[40][50]=1
    map.data[120][100] = 1
    map.data[12][110] = 1
    # map.data[13][10] = 1
    # map.data[13][11] = 1
    # map.data[11][10] = 1

    for row in range(map.info.height):
        map.data[0][row] = 1
        map.data[map.info.width-1][row] = 1
    for col in range(map.info.width):
        map.data[col][0] = 1
        map.data[col][map.info.height -1] = 1
    #https://github.com/z2daj/rbe3002/blob/master/scripts/lab3.py

    print "start"
    self.occupancy_field = OccupancyField(map)
    

  def update_particles_with_odom(self, delta=None):

    delta = delta if delta else (-1, 1, 10)
    print delta

    print "particle cloud with noise"

    #x_sd = delta[0] * .02
    #y_sd = delta[1] * .02
    #theta_sd = delta[2] *.02

    x_sd = .02
    y_sd = .02
    theta_sd = .02

    for particle in self.particle_cloud:
      particle.x += np.random.normal(delta[0], x_sd)
      particle.y += np.random.normal(delta[1], y_sd)
      particle.theta += np.random.normal(delta[2], theta_sd)


  def update_robot_pose(self):
    self.normalize_particles()
    total_x = 0.0
    total_y = 0.0
    total_theta = 0.0
    for particle in self.particle_cloud:
      total_x += particle.x * particle.w
      total_y += particle.y * particle.w
      total_theta += particle.theta * particle.w

    self.pt = (total_x, total_y, total_theta)

  def initialize_particle_cloud(self, xy_theta=None):
    """ Initialize the particle cloud.
      Arguments
      xy_theta: a triple consisting of the mean x, y, and theta (yaw) to initialize the
      particle cloud around.  If this input is ommitted, the odometry will be used """
    if xy_theta == None:
      xy_theta = self.pt
    # TODO(mary): create particles
    x_vals = np.random.normal(xy_theta[0], self.lin_spread, self.n_particles)
    y_vals = np.random.normal(xy_theta[1], self.lin_spread, self.n_particles)
    t_vals = np.random.normal(xy_theta[2], self.ang_spread, self.n_particles)
    self.particle_cloud = [Particle(x_vals[i], y_vals[i], t_vals[i], 1)
                           for i in xrange(self.n_particles)]
    self.normalize_particles()
  
  def normalize_particles(self):
    """ Make sure the particle weights define a valid distribution (i.e. sum to 1.0) """
    # TODO(mary): implement this
    total_weight = sum([particle.w for particle in self.particle_cloud]) * 1.0
    for particle in self.particle_cloud:
      particle.w /= total_weight 
    
  def resample_particles(self):
    """ Resample the particles according to the new particle weights.
    The weights stored with each particle should define the probability that a particular
    particle is selected in the resampling step.  You may want to make use of the given helper
    function draw_random_sample.
    """
    # make sure the distribution is normalized
    self.normalize_particles()
    # TODO: fill out the rest of the implementation
    self.particle_cloud = self.draw_random_sample(
      self.particle_cloud, [particle.w for particle in self.particle_cloud], self.n_particles) 
  
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

  def update_particles_with_laser(self, msg):
    """ Updates the particle weights in response to the scan contained in the msg """
    for particle in self.particle_cloud:
      # print particle.x
      # print particle.y
      closest_particle_object_distance = self.occupancy_field.get_closest_obstacle_distance(particle.x, particle.y) 
      closest_actual_object_distance = msg[0] 
      for i in range(1, 360):
        #readd .range 3 spotd 
        if msg[i] < closest_actual_object_distance:
          closest_actual_object_distance = msg[i]
      print "actual"
      print closest_actual_object_distance
      print "particle"
      print closest_particle_object_distance
      print closest_particle_object_distance-closest_actual_object_distance
      print  self.gauss_particle_probability(closest_particle_object_distance-closest_actual_object_distance)

      particle.w = self.gauss_particle_probability(closest_particle_object_distance-closest_actual_object_distance)

  def gauss_particle_probability(self, difference):
    return (1/(self.variance*self.gauss_constant))*math.exp(-.5*((difference - self.expected_value)/self.variance)**2)
    

def plot(q):
  xs = []
  ys = []
  for p in q.particle_cloud:
    xs.append(p.x)
    ys.append(p.y)
  plt.plot(xs, ys, 'ro')
  plt.plot(q.pt[0], q.pt[1], 'bo')
  a = 10
  i = 0
  plt.axis([i, a, i, a])


q = ParticleFilter()

q.initialize_particle_cloud()

plot(q)
q.update_particles_with_odom()
plot(q)
i=1
print "before"
for particle in q.particle_cloud:
  print i
  i+=1
  print particle.w
msg = []
for i in range(360):
  msg.append(random.uniform(380,1000))
print msg

q.update_particles_with_laser(msg)

i=1
for particle in q.particle_cloud:
  print i
  i+=1
  print particle.w
print "after"
q.resample_particles()
q.normalize_particles()
plt.figure()
q.update_robot_pose()
plot(q)
plt.show()

