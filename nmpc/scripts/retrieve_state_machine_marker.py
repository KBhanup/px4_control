#!/usr/bin/python
from turtle import pos
import rospy as rp
import numpy as np
import quaternion

import threading

from Mag_Eng_DisEng_fns import Mag

from mavros_msgs.msg import RCIn
from px4_control_msgs.msg import DroneStateMarker, Trajectory, Setpoint, MissionState


class StateMachineNode():
    """
      Simple state machine that checks the state and when the target is found it
      sends a setpoint relative to it. It also tracks the target's position and
      if it changes too much it updates the setpoints
    """

    def __init__(self, rate):
        rp.init_node('state_machine_node')
        self.rate = rate

        # Magnet control variables
        self.sensor_magnet_on = Mag(14)
        self.sensor_magnet_off = Mag(15)
        self.drone_magnet = Mag(4)

        # Sensor deployment point relative to marker
        self.H_marker_setpoint = np.array([[1.0, 0.0, 0.0, -0.12],
                                           [0.0, 1.0, 0.0, 0.0],
                                           [0.0, 0.0, 1.0, 0.0],
                                           [0.0, 0.0, 0.0, 1.0]])

        # Marker's pose used for setpoints
        self.marker_position = None
        self.marker_orientation = None

        # Drone's pose and disturbances
        self.drone_position = None
        self.drone_orientation = None
        self.disturbances = None

        # Mission variables
        self.setpoints_initialized = False
        self.publish_setpoint = True
        self.in_mission = False
        self.in_contact = False
        self.wt_sensor = False
        self.mission_bttn = 0
        self.mission_step = 0
        self.z_distances = [-0.95, -0.15, -0.40, -0.75]
        # setpoint: [x, y, z, orientation, z_offset, disturbance]
        self.mission_setpoints = [[0.0, 0.0, 0.0, 0.0, 0.05, None],
                                  [0.0, 0.0, 0.0, 0.0, 0.20, -0.7],
                                  [0.0, 0.0, 0.0, 0.0, 0.20, +0.7],
                                  [0.0, 0.0, 0.0, 0.0, 0.05, None]]

        # Subscribers
        self.state_sub = rp.Subscriber(
            '/drone_state', DroneStateMarker, self.stateCallback, queue_size=1)

        # Publishers
        self.trajectory_pub = rp.Publisher(
            '/drone_trajectory', Trajectory, queue_size=1, latch=True)
        self.mission_state_pub = rp.Publisher(
            '/mission_state', MissionState, queue_size=1)
        self.in_contact_pub = rp.Publisher(
            '/mission_state', MissionState, queue_size=1)

        t = threading.Thread(target=self.missionControl())
        t.start()

        rp.spin()

    """
       Callbacks
    """

    def rcCallback(self, msg):
    # Assign a button on RC to set either Deploy(Down2006) or Retrive(Top982) Mission
        if self.mission_bttn != msg.channels[9]:
            self.mission_bttn = msg.channels[9]
            self.in_mission = True


    def stateCallback(self, msg):
        # Update Drone pose and disturbances
        self.drone_position = np.array([msg.pose.position.x,
                                        msg.pose.position.y,
                                        msg.pose.position.z])

        self.drone_att = np.quaternion(msg.marker_pose.orientation.w,
                                       msg.marker_pose.orientation.x,
                                       msg.marker_pose.orientation.y,
                                       msg.marker_pose.orientation.z).normalized()

        R = quaternion.as_rotation_matrix(self.drone_att)
        self.drone_orientation = np.arctan2(R[1, 0], R[0, 0])

        self.disturbances = np.array([msg.disturbances.x,
                                      msg.disturbances.y,
                                      msg.disturbances.z])
        if msg.marker_found.data:
            if not self.setpoints_initialized:
                marker_att = np.quaternion(msg.marker_pose.orientation.w,
                                           msg.marker_pose.orientation.x,
                                           msg.marker_pose.orientation.y,
                                           msg.marker_pose.orientation.z).normalized()

                self.H_world_marker = np.identity(4)
                self.H_world_marker[0, 3] = msg.marker_pose.position.x
                self.H_world_marker[1, 3] = msg.marker_pose.position.y
                self.H_world_marker[2, 3] = msg.marker_pose.position.z
                self.H_world_marker[0:3, 0:3] = quaternion.as_rotation_matrix(
                    marker_att)

                self.calculateMissionSetpoints()  #(self.H_world_marker)
                self.setpoints_initialized = True

            else:
                marker_current_pos = np.array([msg.marker_pose.position.x,
                                               msg.marker_pose.position.y,
                                               msg.marker_pose.position.z])

                marker_att = np.quaternion(msg.marker_pose.orientation.w,
                                           msg.marker_pose.orientation.x,
                                           msg.marker_pose.orientation.y,
                                           msg.marker_pose.orientation.z).normalized()

                self.H_world_marker = np.identity(4)
                self.H_world_marker[0, 3] = msg.marker_pose.position.x
                self.H_world_marker[1, 3] = msg.marker_pose.position.y
                self.H_world_marker[2, 3] = msg.marker_pose.position.z
                self.H_world_marker[0:3, 0:3] = quaternion.as_rotation_matrix(
                    marker_att)

                R = quaternion.as_rotation_matrix(marker_att)
                marker_current_orientation = np.arctan2(R[1, 0], R[0, 0])

                d_o = marker_current_orientation - self.marker_orientation
                d_p = marker_current_pos - self.marker_position

                if np.dot(d_p, d_p) > 0.02 or abs(d_o) > 0.075:
                    rp.logwarn(
                        'The marker\'s position changed too much. Updating setpoints')

                    self.calculateMissionSetpoints()   #(self.H_world_marker)

    """
       Helper functions
    """

    def calculateMissionSetpoints(self,):
        # Update marker pose
        self.marker_position = np.array([self.H_world_marker[0, 3],
                                         self.H_world_marker[1, 3],
                                         self.H_world_marker[2, 3]])
        self.marker_orientation = np.arctan2(
            self.H_world_marker[1, 0], self.H_world_marker[0, 0])

        for i in range(len(self.z_distances)):
            H_setpoint = self.H_marker_setpoint
            H_setpoint[2, 3] = self.z_distances[i]

            # Transform setpoint to world frame
            H_world_setpoint = np.matmul(self.H_world_marker, H_setpoint)

            self.mission_setpoints[i][0] = H_world_setpoint[0, 3]
            self.mission_setpoints[i][1] = H_world_setpoint[1, 3]
            self.mission_setpoints[i][2] = H_world_setpoint[2, 3]
            self.mission_setpoints[i][3] = np.arctan2(
                H_world_setpoint[1, 0], H_world_setpoint[0, 0])

        # Make sure the updated setpoints are published
        self.publish_setpoint = True

    def sendSetpoint(self,):
        setpoint_msg = Setpoint()
        setpoint_msg.position.x = self.mission_setpoints[self.mission_step][0]
        setpoint_msg.position.y = self.mission_setpoints[self.mission_step][1]
        setpoint_msg.position.z = self.mission_setpoints[self.mission_step][2]
        setpoint_msg.velocity.x = 0.0
        setpoint_msg.velocity.y = 0.0
        setpoint_msg.velocity.z = 0.0
        setpoint_msg.orientation.x = 0.0
        setpoint_msg.orientation.y = 0.0
        setpoint_msg.orientation.z = self.mission_setpoints[self.mission_step][3]

        trajectory_msg = Trajectory()
        trajectory_msg.header.stamp = rp.Time.now()
        trajectory_msg.trajectory.append(setpoint_msg)

        self.trajectory_pub.publish(trajectory_msg)

    def sendMissionState(self,):
        mission_state_msg = MissionState()
        mission_state_msg.in_contact.data = self.in_contact
        mission_state_msg.wt_sensor.data = self.wt_sensor
        self.mission_state_pub(mission_state_msg)

    def checkState(self,):
        dx = abs(self.drone_position[0] -
                 self.mission_setpoints[self.mission_step][0])
        dy = abs(self.drone_position[1] -
                 self.mission_setpoints[self.mission_step][1])
        dz = abs(self.drone_position[2] -
                 self.mission_setpoints[self.mission_step][2])
        do = abs(self.drone_orientation -
                 self.mission_setpoints[self.mission_step][3])

        pose_condition = dx < 0.03 and \
            dy < 0.03 and \
            dz < self.mission_setpoints[self.mission_step][4] and \
            do < 0.075

        if self.mission_setpoints[self.mission_step][5] is None:
            return pose_condition
        else:
            dist_condition = abs(
                self.mission_setpoints[self.mission_step][5] + self.disturbances[2]) > 1.4
            return pose_condition and dist_condition

    """
       State Machine main function
    """

    def stateMachine(self,):
        # Publish setpoint
        if self.publish_setpoint:
            self.sendSetpoint()
            self.publish_setpoint = False

        if self.checkState():
            rp.loginfo('Reached setpoint {}'.format(self.mission_step))

            if self.mission_step == 1:
                self.in_contact = True
                rp.loginfo('Engaging drone magnet')
                self.drone_magnet.dr_Magengage()

            elif self.mission_step == 2:
                rp.loginfo('Disengaging sensor magnet')
                self.sensor_magnet_off.Sn_Magdisengage()
                rp.loginfo('Sensor retrieved')
                self.in_contact = False
                self.wt_sensor = True

            elif self.mission_step == 3:
                rp.loginfo('Retrieve mission finished')
                self.in_mission = False

            self.mission_step += 1
            self.publish_setpoint = True

    """
       Main mission function
    """

    def missionControl(self,):
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            # Publish in contact
            self.sendMissionState()

            # Run state machine
            if self.in_mission and self.setpoints_initialized:
                self.stateMachine()

            r.sleep()


if __name__ == '__main__':
    StateMachineNode(5)
