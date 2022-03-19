from ast import Num
from datetime import date
from math import sqrt
import numpy as np
from scan_simulator_2d import PyScanSimulator2D

import rospy
import tf
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler

class SensorModel:


    def __init__(self):
        # Fetch parameters
        self.map_topic = rospy.get_param("~map_topic")
        # self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
        self.num_beams_per_particle = rospy.get_param("~num_beams_per_particle")
        self.scan_theta_discretization = rospy.get_param("~scan_theta_discretization")
        self.scan_field_of_view = rospy.get_param("~scan_field_of_view")
        self.LIDAR_TO_MAP_SCALE = rospy.get_param("~lidar_scale_to_map_scale")

        ####################################mt
        # TODO
        # Adjust these parameters
        self.alpha_hit = 0.74
        self.alpha_short = 0.07
        self.alpha_max = 0.07
        self.alpha_rand = 0.12
        self.sigma_hit = 8.0

        # Your sensor table will be a `table_width` x `table_width` np array:
        self.table_width = 201
        self.z_max = 200
    
        ####################################

        # Precompute the sensor model table
        self.sensor_model_table = None  #z by d 
        self.precompute_sensor_model()

        # Create a simulated laser scan
        self.scan_sim = PyScanSimulator2D(
                self.num_beams_per_particle,
                self.scan_field_of_view,
                0, # This is not the simulator, don't add noise
                0.01, # This is used as an epsilon
                self.scan_theta_discretization) 
        self.resolution = None
        # Subscribe to the map
        self.map = None
        self.map_set = False
        rospy.Subscriber(
                self.map_topic,
                OccupancyGrid,
                self.map_callback,
                queue_size=1)

    def precompute_sensor_model(self):
        """
        Generate and store a table which represents the sensor model.
        
        For each discrete computed range value, this provides the probability of 
        measuring any (discrete) range. This table is indexed by the sensor model
        at runtime by discretizing the measurements and computed ranges from
        RangeLibc.
        This table must be implemented as a numpy 2D array.

        Compute the table based on class parameters alpha_hit, alpha_short,
        alpha_max, alpha_rand, sigma_hit, and table_width.

        args:
            N/A
        
        returns:
            No return type. Directly modify `self.sensor_model_table`.
        """
        z = np.linspace(0, self.z_max, self.table_width) 
        z = np.repeat(np.expand_dims(z,0), self.table_width, 0).T

        d = np.linspace(0, self.z_max, self.table_width)
        d = d.astype(float)
        d = np.repeat(np.expand_dims(d,0), self.table_width, 0)

        # compute matrices
        p_hit = (1/sqrt(2*np.pi*(self.sigma_hit)**2))*np.exp(-(z-d)**2/(2*self.sigma_hit**2)).T
        

        #Modified to avoid divide by zero error
        check_z = z[np.logical_and(z <= d, d > 0)]
        check_d = d[np.logical_and(z <= d, d > 0)]
        p_short = np.zeros_like(p_hit)
        p_short[np.logical_and(z <= d, d > 0)] = (2.0/check_d) * (1.0 - (check_z/check_d))

        # p_short = np.where((z <= d), (2/(d))*(1-(z/(d))), 0)
        # p_short = np.where(d==0, 0, p_short)

        p_max = np.where(z == self.z_max, 1, 0)
        p_rand = np.ones((self.table_width, self.table_width))/self.z_max

        # normalize p_hit 
        for i in range(self.table_width):
            p_hit[i, :] = p_hit[i, :]/np.sum(p_hit[i, :])

        self.sensor_model_table = (self.alpha_hit*p_hit.T + self.alpha_short*p_short + self.alpha_max*p_max + self.alpha_rand*p_rand).T

        for i in range(self.table_width):
            self.sensor_model_table[i, :] = self.sensor_model_table[i, :]/np.sum(self.sensor_model_table[i, :])
        self.sensor_model_table = self.sensor_model_table.T

        return None

    def evaluate(self, particles, observation):
        """
        Evaluate how likely each particle is given
        the observed scan.

        args:
            particles: An Nx3 matrix of the form:
            
                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            observation: A vector of lidar data measured
                from the actual lidar.

        returns:
           probabilities: A vector of length N representing
               the probability of each particle existing
               given the observation and the map.
        """

        if not self.map_set:
            return

        ####################################
        # TODO
        # Evaluate the sensor model here!
        #
        # You will probably want to use this function
        # to perform ray tracing from all the particles.
        # This produces a matrix of size N x num_beams_per_particle 

        scans = self.scan_sim.scan(particles) # ground truth 

        ####################################
        scans = scans/(float(self.resolution)*self.LIDAR_TO_MAP_SCALE)
        observation = observation/(float(self.resolution)*self.LIDAR_TO_MAP_SCALE)

        #clip anything > zmax to zmax and anything <0 to 0
        scans = np.where(scans > self.z_max, self.z_max, scans)
        scans = np.where(scans < 0, 0, scans)
        observation = np.where(observation > self.z_max, self.z_max, observation)
        observation = np.where(observation < 0, 0, observation)

        d_vals = np.rint(scans).astype(np.uint16)
        z_vals = np.rint(observation).astype(np.uint16)
        p_array = self.sensor_model_table[z_vals, d_vals]
        probabilities = np.power(np.prod(p_array, axis=1), 1/2.2)

        # probabilities = []
        # for particle_scan in scans:
        #     p_i = 1
        #     for i in range(len(particle_scan)): # scan ground truth, aka d 
        #         d_val = int(particle_scan[i])
        #         z_val= int(observation[i])
        #         p_i *= self.sensor_model_table[z_val][d_val]
        # probabilities.append(p_i**(1/2.2))

        return probabilities


    def map_callback(self, map_msg):
        # Convert the map to a numpy array
        self.map = np.array(map_msg.data, np.double)/100.
        self.map = np.clip(self.map, 0, 1)

        # Convert the origin to a tuple
        origin_p = map_msg.info.origin.position
        origin_o = map_msg.info.origin.orientation
        origin_o = tf.transformations.euler_from_quaternion((
                origin_o.x,
                origin_o.y,
                origin_o.z,
                origin_o.w))
        origin = (origin_p.x, origin_p.y, origin_o[2])

        # Initialize a map with the laser scan
        self.scan_sim.set_map(
                self.map,
                map_msg.info.height,
                map_msg.info.width,
                map_msg.info.resolution,
                origin,
                0.5) # Consider anything < 0.5 to be free
        
        self.resolution = map_msg.info.resolution

        # Make the map set
        self.map_set = True

        print("Map initialized")
