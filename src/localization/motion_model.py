import numpy as np 
from math import cos, sin, atan2, acos
import rospy

class MotionModel:

    def __init__(self):
        # self.ODOM_TOPIC 

        ####################################
        # TODO
        # Do any precomputation for the motion
        # model here.

        # rospy.get_param()
        # sub = rospy.Subscriber(self.ODOM_TOPIC, LaserScan, self.callback)
        pass

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

        # for i in range(N):
        #     odom_T = self.get_rotation_mat(odometry[0], odometry[1], odometry[2])
        #     particle_T = self.get_rotation_mat(particles[i, 0], particles[i, 1], particles[i, 2])
        #     # result[i, :] = np.dot(particles[i, :], odom_T) #(1,3) * (3,3) = (1,3)
        #     future_particle_T =  np.dot(particle_T, odom_T) #(3,3) * (3,3) = (3,3)
        #     #theta = acos(future_particle_T[1,1])
        #     future_theta = (particles[i,2]) + odometry[2]
        #     result[i, :] = np.array([[future_particle_T[0,2], future_particle_T[1,2], future_theta]]) #self.get_state(future_particle_T, odometry[2])
        ####################################
        return result

    # def get_rotation_mat(self, x, y, theta):
    #     #return 1x3
    #     result = np.zeros((3,3))
    #     result[0,0] = cos(theta)
    #     result[0,1] = -sin(theta)
    #     result[1,0] = sin(theta)
    #     result[1,1] = cos(theta)

    #     result[0,2] = x
    #     result[1,2] = y
    #     result[2,2] = 1
    #     return result

    # def get_state(self, mat, dtheta):
    #     theta = acos(mat[1,0]) #atan2(mat[1,0], mat[0,0])
    #     # theta += 
    #     theta += dtheta
    #     return np.array([[mat[0,2], mat[1,2], theta]])
