#!/usr/bin/env python

import rospy
import math
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose

# Global variables
particles = []
num_particles = 100
alpha1, alpha2, alpha3, alpha4 = 0.001, 0.001, 0.001, 0.001

def odom_callback(msg):
    global particles
    if not particles:
        # Initialize particles if empty
        for _ in range(num_particles):
            particles.append([msg.pose.pose.position.x, msg.pose.pose.position.y, get_yaw(msg.pose.pose.orientation)])
        return

    new_particles = []
    for particle in particles:
        # Calculate delta values
        delta_rot1 = math.atan2(msg.pose.pose.position.y - particle[1], msg.pose.pose.position.x - particle[0]) - particle[2]
        delta_trans = math.sqrt((msg.pose.pose.position.x - particle[0]) ** 2 + (msg.pose.pose.position.y - particle[1]) ** 2)
        delta_rot2 = get_yaw(msg.pose.pose.orientation) - particle[2] - delta_rot1

        # Add noise
        delta_rot1_hat = delta_rot1 + sample(alpha1 * delta_rot1 ** 2 + alpha2 * delta_trans ** 2)
        delta_trans_hat = delta_trans + sample(alpha3 * delta_trans ** 2 + alpha4 * (delta_rot1 ** 2 + delta_rot2 ** 2))
        delta_rot2_hat = delta_rot2 + sample(alpha1 * delta_rot2 ** 2 + alpha2 * delta_trans ** 2)

        # Update particle
        new_x = particle[0] + delta_trans_hat * math.cos(particle[2] + delta_rot1_hat)
        new_y = particle[1] + delta_trans_hat * math.sin(particle[2] + delta_rot1_hat)
        new_theta = particle[2] + delta_rot1_hat + delta_rot2_hat

        new_particles.append([new_x, new_y, new_theta])

    particles = new_particles
    publish_particles()

def get_yaw(orientation):
    siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
    return math.atan2(siny_cosp, cosy_cosp)

def sample(b):
    return math.sqrt(6) / 2 * (random.uniform(-b, b) + random.uniform(-b, b))

def publish_particles():
    global particles
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = "odom"
    for particle in particles:
        pose = Pose()
        pose.position.x = particle[0]
        pose.position.y = particle[1]
        pose.orientation.z = math.sin(particle[2] / 2.0)
        pose.orientation.w = math.cos(particle[2] / 2.0)
        pose_array.poses.append(pose)
    pub.publish(pose_array)

if __name__ == '__main__':
    rospy.init_node('particle_filter')
    pub = rospy.Publisher('particle_filter/particles', PoseArray, queue_size=10)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.spin()
