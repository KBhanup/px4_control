#!/usr/bin/python3
import rospy as rp
import numpy as np
import random

from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry


class FakeGPS():
    """
      Relays the position from the local_position/odom msg after adding noise:

      p_fake = p_odom + random_walk + noise

      Does not affect the orientation
    """

    def __init__(self, random_walk_variance, noise_variance):
        rp.init_node('fake_gps_node')

        # Noise setup
        self.random_walk_std = pow(random_walk_variance, 0.5)
        self.noise_std = pow(noise_variance, 0.5)
        self.random_walk = Vector3Stamped()
        self.random_walk.vector.x = 0.0
        self.random_walk.vector.y = 0.0
        self.random_walk.vector.z = 0.0

        # Subscribers
        self.source_pose_sub = rp.Subscriber(
            '/mavros/local_position/odom', Odometry, self.poseCallback, queue_size=1)

        # Publishers
        self.noisy_pose_pub = rp.Publisher(
            '/fake_gps/pose', Odometry, queue_size=1)

        self.random_bias_pub = rp.Publisher(
            '/fake_gps/random_bias', Vector3Stamped, queue_size=1)

        rp.spin()

    def poseCallback(self, msg):
        # Update random walk noise
        self.random_walk.vector.x += random.gauss(0.0, self.random_walk_std)
        self.random_walk.vector.y += random.gauss(0.0, self.random_walk_std)
        self.random_walk.vector.z += random.gauss(0.0, self.random_walk_std)

        # Prepare messages
        fake_msg = Odometry()
        fake_msg.header.stamp = msg.header.stamp
        fake_msg.header.frame_id = msg.header.frame_id
        fake_msg.pose.pose.position.x = msg.pose.pose.position.x + \
            self.random_walk.vector.x + random.gauss(0.0, self.noise_std)
        fake_msg.pose.pose.position.y = msg.pose.pose.position.y + \
            self.random_walk.vector.y + random.gauss(0.0, self.noise_std)
        fake_msg.pose.pose.position.z = msg.pose.pose.position.z + \
            self.random_walk.vector.z + random.gauss(0.0, self.noise_std)
        fake_msg.pose.pose.orientation = msg.pose.pose.orientation

        self.random_walk.header.stamp = msg.header.stamp

        self.noisy_pose_pub.publish(fake_msg)
        self.random_bias_pub.publish(self.random_walk)



if __name__ == '__main__':
    FakeGPS(1.0e-6, 1.0e-4)
