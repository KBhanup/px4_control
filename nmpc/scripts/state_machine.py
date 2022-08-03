#!/usr/bin/python3
import rospy as rp
import numpy as np
import quaternion

import threading

from px4_control_msgs.msg import DroneStateMarker, Setpoint, Trajectory


class StateMachineNode():
    """
      Simple state machine that checks the state and when the target is found it
      sends a setpoint relative to it. It also tracks the target's position and
      if it changes too much it updates the setpoint
    """

    def __init__(self, rate):
        rp.init_node('state_machine_node')
        self.rate = rate

        # Check that NMPC is running by checking that a service is available
        rp.loginfo('Checking that controller is up')
        rp.wait_for_service('/enable_controller')
        rp.loginfo('NMPC is up')

        # Setpoint
        self.H_marker_setpoint = np.array([[1.0, 0.0, 0.0, -4.0],
                                           [0.0, 1.0, 0.0,  0.0],
                                           [0.0, 0.0, 1.0,  0.0],
                                           [0.0, 0.0, 0.0,  1.0]])
        self.marker_setpoint_sent = False
        self.marker_position = None
        self.marker_orientation = None
        self.traj = Trajectory()

        # Subscribers
        self.state_sub = rp.Subscriber(
            '/drone_state', DroneStateMarker, self.stateCallback, queue_size=1)

        # Publishers
        self.trajectory_pub = rp.Publisher(
            '/drone_trajectory', Trajectory, queue_size=1, latch=True)
        self.traj_logs_pub = rp.Publisher(
            '/drone_trajectory_log', Trajectory, queue_size=1)

        t = threading.Thread(target=self.trajPublisher)
        t.start()

        rp.spin()

    def stateCallback(self, msg):
        if msg.marker_found.data:
            if not self.marker_setpoint_sent:
                # Get marker world position
                self.marker_position = np.array([msg.marker_pose.position.x,
                                                 msg.marker_pose.position.y,
                                                 msg.marker_pose.position.z])

                marker_att = np.quaternion(msg.marker_pose.orientation.w,
                                           msg.marker_pose.orientation.x,
                                           msg.marker_pose.orientation.y,
                                           msg.marker_pose.orientation.z).normalized()

                H_world_marker = np.identity(4)
                H_world_marker[0, 3] = msg.marker_pose.position.x
                H_world_marker[1, 3] = msg.marker_pose.position.y
                H_world_marker[2, 3] = msg.marker_pose.position.z
                H_world_marker[0:3, 0:3] = quaternion.as_rotation_matrix(
                    marker_att)

                self.marker_orientation = np.arctan2(
                    H_world_marker[1, 0], H_world_marker[0, 0])

                # Transform setpoint to world frame
                H_world_setpoint = np.matmul(
                    H_world_marker, self.H_marker_setpoint)

                # Prepare setpoint message and publish it
                setpoint_msg = Setpoint()
                setpoint_msg.position.x = H_world_setpoint[0, 3]
                setpoint_msg.position.y = H_world_setpoint[1, 3]
                setpoint_msg.position.z = H_world_setpoint[2, 3]
                setpoint_msg.velocity.x = 0.0
                setpoint_msg.velocity.y = 0.0
                setpoint_msg.velocity.z = 0.0
                setpoint_msg.orientation.x = 0.0
                setpoint_msg.orientation.y = 0.0
                setpoint_msg.orientation.z = np.arctan2(
                    H_world_setpoint[1, 0], H_world_setpoint[0, 0])

                self.traj.header.stamp = rp.Time.now()
                self.traj.trajectory.append(setpoint_msg)
                self.trajectory_pub.publish(self.traj)

                rp.loginfo('Setpoint sent')
                self.marker_setpoint_sent = True
            else:
                marker_current_pos = np.array([msg.marker_pose.position.x,
                                               msg.marker_pose.position.y,
                                               msg.marker_pose.position.z])

                marker_att = np.quaternion(msg.marker_pose.orientation.w,
                                           msg.marker_pose.orientation.x,
                                           msg.marker_pose.orientation.y,
                                           msg.marker_pose.orientation.z).normalized()
                R = quaternion.as_rotation_matrix(marker_att)

                marker_current_orientation = np.arctan2(R[1, 0], R[0, 0])

                d_o = marker_current_orientation - self.marker_orientation
                d_p = marker_current_pos - self.marker_position

                if np.dot(d_p, d_p) > 0.02 or abs(d_o) > 0.075:
                    rp.logwarn(
                        'The marker\'s position changed too much. Sending new setpoint')
                    self.traj.trajectory = []
                    self.marker_setpoint_sent = False

    def trajPublisher(self,):
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            if self.marker_setpoint_sent:
                self.traj.header.stamp = rp.Time.now()
                self.traj_logs_pub.publish(self.traj)

            r.sleep()


if __name__ == '__main__':
    StateMachineNode(2)
