#!/usr/bin/env python2

import rospy
from sensor_model import SensorModel
from motion_model import MotionModel

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from math import atan2, cos, sin
from scipy.spatial.transform import Rotation as R

import numpy as np

class ParticleFilter:
    def __init__(self):
        # OUR PARAMETERS
        self.NUMBER_OF_PARTICLES = rospy.get_param("~num_particles", 500) #200
        self.particles = np.zeros((self.NUMBER_OF_PARTICLES, 3))
        # self.odometry = np.zeros((3,)) #[dx, dy, dtheta]
        # self.observation = #vector of lidar data; TODO: unknown size? 


        # Get parameters
        self.particle_filter_frame = \
                rospy.get_param("~particle_filter_frame")

        # Initialize publishers/subscribers
        #
        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        scan_topic = rospy.get_param("~scan_topic", "/scan")
        odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.laser_sub = rospy.Subscriber(scan_topic, LaserScan,
                                          self.lidar_callback, # TODO: Fill this in
                                          queue_size=1)
        self.odom_sub  = rospy.Subscriber(odom_topic, Odometry,
                                          self.odom_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.
        self.pose_sub  = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped,
                                          self.pose_initialization_callback, # TODO: Fill this in
                                          queue_size=1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.
        self.odom_pub  = rospy.Publisher("/pf/pose/odom", Odometry, queue_size = 1)
        
        # Initialize the models
        self.motion_model = MotionModel(deterministic=rospy.get_param("~deterministic", True))
        self.sensor_model = SensorModel()

        self.transform_broadcaster = TransformBroadcaster()
        
        self.lasttime_odom = 0

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.
        # self.calculate_average_and_send_transform()
    
    def lidar_callback(self, lidar_scan):
        #update particles with sensor model
        observation = np.array(lidar_scan.ranges) #TODO: do we need to do further filtering?
        # print("observation:", np.array(observation).shape)
        particle_probabilities = self.sensor_model.evaluate(self.particles, observation) #vector of length N
        
        if particle_probabilities is not None:
            particle_probabilities = particle_probabilities/np.sum(particle_probabilities)
            # print("particle_probabilities:", np.sum(particle_probabilities))
            resampled_particles_indices = np.random.choice(self.NUMBER_OF_PARTICLES, (self.NUMBER_OF_PARTICLES,), p=particle_probabilities)
            self.particles = self.particles[resampled_particles_indices, :]
            #self.particles = np.unique(self.partices, axis=0)

            self.calculate_average_and_send_transform()
        

    def odom_callback(self, odometry):
        #update particles with motion model
        
        time = odometry.header.stamp.secs
        dt = time-self.lasttime_odom
        
        if (dt > 1.0):
            self.lasttime_odom = time
        else:
            #TODO: idk if odometry calculation is right
            
            dtheta = odometry.twist.twist.angular.z*dt
            u = np.array([odometry.twist.twist.linear.x*dt, odometry.twist.twist.linear.y*dt, dtheta])
    
            self.particles = self.motion_model.evaluate(self.particles, u)
    
            self.calculate_average_and_send_transform()
            
            self.lasttime_odom = time

    def pose_initialization_callback(self, data):
        init_x, init_y = data.pose.pose.position.x, data.pose.pose.position.y
        q = data.pose.pose.orientation
        cov = data.pose.covariance
        r = R.from_quat([q.x, q.y, q.z, q.w])
        init_theta = r.as_euler('XYZ')[2]

        self.init_data = np.array([init_x, init_y, init_theta])

        self.particles = np.random.normal(self.init_data, np.sqrt(np.array([cov[0], cov[6 + 1], cov[6 * 5 + 5]])), (self.NUMBER_OF_PARTICLES, 3))

        #print(self.particles)
        print(self.init_data)

    def get_3d_rot_matrix(self, x, y, theta):
        result = np.zeros((3,3))
        result[0,0] = cos(theta)
        result[0,1] = -sin(theta)
        result[1,0] = sin(theta)
        result[1,1] = cos(theta)

        result[0,2] = x
        result[1,2] = y
        result[2,2] = 1
        return result

    def calculate_average_and_send_transform(self):
        # If we hit a bimodal distribution, how do we deal with it?
        xy_mean = np.mean(self.particles[:, 0:2], axis=0) #(2,)
        mean_sin = np.mean(np.sin(self.particles[:, 2:3]), axis=0) #(1,)   #, keepdims=True)
        mean_cos = np.mean(np.cos(self.particles[:, 2:3]), axis=0) #(1,)   #, keepdims=True)
        mean_theta = atan2(mean_sin.item(), mean_cos.item())
        #mean_particles = np.array([xy_mean[0], xy_mean[1], mean_theta])

        #print(self.particles[:, 0:2])
        #print(xy_mean, mean_theta)

        #TODO: check transform is correct
        t = TransformStamped()

        q = R.from_euler('Z', mean_theta, degrees=False).as_quat()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "/map"
        t.child_frame_id = self.particle_filter_frame #TODO: generalize for simulator & car
        t.transform.translation.x = xy_mean[0]
        t.transform.translation.y = xy_mean[1]
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.transform_broadcaster.sendTransform(t)

        odom_msg = Odometry()
        odom_msg.header.frame_id = "/map"
        odom_msg.pose.pose.position.x = xy_mean[0]
        odom_msg.pose.pose.position.y = xy_mean[1]
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(odom_msg)


if __name__ == "__main__":
    rospy.init_node("particle_filter")
    pf = ParticleFilter()
    rospy.spin()
