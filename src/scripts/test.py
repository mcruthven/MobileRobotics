import numpy as np
from copy import deepcopy
import random
from numpy.random import random_sample

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
q.resample_particles()
q.normalize_particles()
plt.figure()
q.update_robot_pose()
plot(q)
plt.show()

