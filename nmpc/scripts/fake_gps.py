#!/usr/bin/python3
import rospy as rp
import numpy as np
import random

from geometry_msgs.msg import PoseStamped


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
        self.random_walk = np.zeros(3)
        self.noise_std = pow(noise_variance, 0.5)

        # Subscribers
        self.source_pose_sub = rp.Subscriber(
            '/mavros/local_position/odom', PoseStamped, self.poseCallback, queue_size=1)

        # Publishers
        self.noisy_pose_pub = rp.Publisher(
            'noisy_pose', PoseStamped, queue_size=1)

        rp.spin()

    def mocapCallback(self, msg):
        # Update random walk noise
        self.random_walk[0] += random.gauss(0.0, self.random_walk_std)
        self.random_walk[1] += random.gauss(0.0, self.random_walk_std)
        self.random_walk[2] += random.gauss(0.0, self.random_walk_std)

        # Prepare message
        fake_msg = PoseStamped()
        fake_msg.header.stamp = msg.header.stamp
        fake_msg.header.frame_id = msg.header.frame_id
        fake_msg.pose.position.x = msg.pose.position.x + \
            self.random_walk[0] + random.gauss(0.0, self.noise_std)
        fake_msg.pose.position.y = msg.pose.position.y + \
            self.random_walk[1] + random.gauss(0.0, self.noise_std)
        fake_msg.pose.position.z = msg.pose.position.z + \
            self.random_walk[2] + random.gauss(0.0, self.noise_std)
        fake_msg.pose.orientation = msg.pose.orientation

        self.noisy_pose_pub.publish(fake_msg)


if __name__ == '__main__':
    FakeGPS(1.0e-6, 1.0e-4)
