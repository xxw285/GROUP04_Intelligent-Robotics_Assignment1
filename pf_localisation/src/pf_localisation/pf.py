import numpy as np
from geometry_msgs.msg import Pose, PoseArray, Quaternion, PoseWithCovarianceStamped
from . pf_base import PFLocaliserBase
import math
import rospy

from . util import rotateQuaternion, getHeading
from random import random

from time import time


class PFLocaliser(PFLocaliserBase):
       
    def __init__(self):
        # ----- Call the superclass constructor
        super(PFLocaliser, self).__init__()
        
        # ----- Set model noise value
        self.ODOM_ROTATION_NOISE = 0.1  # Odometry model rotation noise
        self.ODOM_TRANSLATION_NOISE = 1  # Odometry x axis (forward) noise
        self.ODOM_DRIFT_NOISE = 1  # Odometry y axis (side-side) noise
        # ----- Sensor the number of particles
        self.NUMBER_PREDICTED_READINGS = 200     # Number of readings to predict

       
    def initialise_particle_cloud(self, init_Pose):
        """
        Set particle cloud to initialpose plus noise

        Called whenever an initialpose message is received (to change the
        starting location of the robot), or a new occupancy_map is received.
        self.particlecloud can be initialised here. Initial pose of the robot
        is also set here.
        
        :Args:
            | initialpose: the initial pose estimate
        :Return:
            | (geometry_msgs.msg.PoseArray) poses of the particles
        """
        #initialize position and orientation values
        init_particile_x = init_Pose.pose.pose.position.x
        init_particile_y = init_Pose.pose.pose.position.y
        init_particile_z = init_Pose.pose.pose.position.z
        init_particile_o = init_Pose.pose.pose.orientation


        init_particle = PoseArray()

        #initialize particle values
        for num in range(self.NUMBER_PREDICTED_READINGS):
            estimatedpose = PoseWithCovarianceStamped()
            estimatedpose.pose.pose.position.x = np.random.normal(0, 0.1, 1) * self.ODOM_TRANSLATION_NOISE + init_particile_x
            estimatedpose.pose.pose.position.y = np.random.normal(0, 0.1, 1) * self.ODOM_DRIFT_NOISE + init_particile_y
            estimatedpose.pose.pose.position.z = init_particile_z

            var_angle = np.random.normal(0, 0.1, 1) * self.ODOM_ROTATION_NOISE  # change the distribution values
            estimatedpose.pose.pose.orientation = rotateQuaternion(init_particile_o, var_angle)

            # append the position and orientation information of each particle into the particle array
            init_particle.poses.append(Pose(estimatedpose.pose.pose.position, estimatedpose.pose.pose.orientation))

        return init_particle
        #pass

 
    
    def update_particle_cloud(self, scan):



        def update_particle_cloud(self, scan):
            """
            This should use the supplied laser scan to update the current
            particle cloud. i.e. self.particlecloud should be updated.

            :Args:
                | scan (sensor_msgs.msg.LaserScan): laser scan to use for update
             """
            #initilize a blank array for storing particles' weights and the sum of weights
            particle_weight = []
            sum = 0
            #scan the particle's weight and sum them up
            for p in self.particlecloud.poses:
                w = self.sensor_model.get_weight(scan, p)
                sum += w
                particle_weight.append(w)

            #initialize weights
            for x in range(self.NUMBER_PREDICTED_READINGS):
                particle_weight[x] = particle_weight[x] / sum

            #resampling
            particle_weight.sort()#sort the array in ascending order
            res_particle = PoseArray()# initialze an array for particle cloud
            m = 1 / self.NUMBER_PREDICTED_READINGS
            random_r = np.random.uniform(0, 1 / self.NUMBER_PREDICTED_READINGS)
            num = 0
            c = 0
            u = 0
            # start resampling particles
            for x in range(self.NUMBER_PREDICTED_READINGS):
                u = random_r + (x-1) * m
                while u > c:
                    num += + 1
                    c += particle_weight[x]
                res_particle.poses.append(self.particlecloud.poses[num])
            self.particlecloud = res_particle

    def estimate_pose(self):
        """
        This should calculate and return an updated robot pose estimate based
        on the particle cloud (self.particlecloud).
        
        Create new estimated pose, given particle cloud
        E.g. just average the location and orientation values of each of
        the particles and return this.
        
        Better approximations could be made by doing some simple clustering,
        e.g. taking the average location of half the particles after 
        throwing away any which are outliers

        :Return:
            | (geometry_msgs.msg.Pose) robot's estimated pose.
         """
        #initialize the pose covariance
        sum_x = 0
        sum_y = 0
        sum_z = 0
        sum_o = 0
        #calculate the sum of each particle's position:x, y, z and orientation
        for i in range(self.NUMBER_PREDICTED_READINGS):
            sum_x += self.particlecloud.poses[i].position.x
            sum_y += self.particlecloud.poses[i].position.y
            sum_z = self.particlecloud.poses[i].position.z

            # convert the degree into radius
            temp_o = getHeading(self.particlecloud.poses[i].orientation)
            # if the angle is negative, add 2pi to make it positive
            if temp_o < 0:
                temp_o += 2 * math.pi
            sum_o += temp_o
        #calculate the average position values
        aver_x = sum_x / self.NUMBER_PREDICTED_READINGS
        aver_y = sum_y / self.NUMBER_PREDICTED_READINGS
        aver_z = sum_z / self.NUMBER_PREDICTED_READINGS
        aver_o = sum_o / self.NUMBER_PREDICTED_READINGS

        #assign the position values of the robot
        est_particle = PoseWithCovarianceStamped()
        est_particle.pose.pose.position.x = aver_x
        est_particle.pose.pose.position.y = aver_y
        est_particle.pose.pose.position.z = aver_z
        est_particle.pose.pose.orientation = rotateQuaternion(Quaternion(w=1.0),aver_o)


        return Pose(est_particle.pose.pose.position, est_particle.pose.pose.orientation)



        #pass
