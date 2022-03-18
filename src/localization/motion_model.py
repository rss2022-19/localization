import numpy as np 
from math import cos, sin, atan2, acos
import rospy

class MotionModel:

    def __init__(self, deterministic=False):
        # self.ODOM_TOPIC 

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.
        # pass

        self.deterministic = True #deterministic
                                   # set True to past unit test
        ####################################

    def evaluate(self, particles, odometry):
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """
        
        ####################################
        N = particles.shape[0]
        result = np.zeros_like(particles)
        result[:, 0] = odometry[0]*np.cos(particles[:, 2]) + odometry[1]*-np.sin(particles[:, 2]) + particles[:, 0]
        result[:, 1] = odometry[0]*np.sin(particles[:, 2]) + odometry[1]*np.cos(particles[:, 2]) + particles[:, 1]
        result[:, 2] = odometry[2] + particles[:, 2]
        
        #Add gaussian noise
        self.x_spread = rospy.get_param("x_spread", 1.)
        self.y_spread = rospy.get_param("y_spread", 1.)
        self.theta_spread = rospy.get_param("theta_spread", 3.14/2)
        
        result[:, 0] += np.clip(np.random.normal(0, self.x_spread, N), -self.x_spread*3,self.x_spread*3)
        result[:, 1] += np.clip(np.random.normal(0, self.y_spread, N), -self.y_spread*3,self.y_spread*3)
        result[:, 2] += np.clip(np.random.normal(0, self.theta_spread, N), -self.theta_spread*3,self.theta_spread*3)

        perturb = np.zeros_like(result)
        if self.deterministic == False:
            x_var = 1
            y_var = 1
            theta_var = np.pi/100 #TODO: change variances?
            
            perturb[:, 0:2] = np.random.normal(0, 1, size=(N,2))
            perturb[:, 2:3] = np.random.normal(0, 1, size=(N,1))
        
            result += perturb

        return result