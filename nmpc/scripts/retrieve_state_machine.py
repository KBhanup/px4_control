#!/usr/bin/python
import rospy as rp
import numpy as np
import quaternion

import threading

from magnet_control import MagnetControl

from geometry_msgs.msg import Vector3
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
        self.drone_magnet = MagnetControl(4)
        self.sensor_magnet_on = MagnetControl(14)
        self.sensor_magnet_off = MagnetControl(15)

        # Sensor deployment point relative to marker
        deployed_x = -0.15
        deployed_y = 0.02
        self.H_marker_setpoint = np.array([[1.0, 0.0, 0.0, deployed_x],
                                           [0.0, 1.0, 0.0, deployed_y],
                                           [0.0, 0.0, 1.0,  0.00],
                                           [0.0, 0.0, 0.0,  1.00]])
        # When the drone with the sensor are in contact with the ceiling, the distance
        # from the ceiling is around -0.25m. For deploying set the distance to +-0.15
        # so that disturbances are properly formed
        self.z_distances = [-0.65, -0.20, -0.45, -0.65]

        # Marker's pose used for setpoints
        self.marker_position = None
        self.marker_orientation = None

        # Drone's pose and disturbances
        self.drone_position = None
        self.drone_orientation = None
        self.disturbances = None

        # Mission variables
        self.mission_bttn = 0
        self.setpoints_initialized = False
        self.publish_setpoint = True
        self.in_mission = False

        # Mission State
        self.in_contact = False
        self.wt_sensor = False

        self.mission_start_t = None
        self.mission_step = 0
        self.mission_setpoints = [{'set_x': 0.0, 'set_y': 0.0, 'set_z': 0.0, 'set_o': 0.0,
                                   'hor_offset': 0.02, 'ver_offset': 0.05, 'required_force': None},
                                  {'set_x': 0.0, 'set_y': 0.0, 'set_z': 0.0, 'set_o': 0.0,
                                   'hor_offset': 0.02, 'ver_offset': 0.20, 'required_force': -0.7},
                                  {'set_x': 0.0, 'set_y': 0.0, 'set_z': 0.0, 'set_o': 0.0,
                                   'hor_offset': 1.00, 'ver_offset': 0.20, 'required_force':  0.7},
                                  {'set_x': 0.0, 'set_y': 0.0, 'set_z': 0.0, 'set_o': 0.0,
                                   'hor_offset': 0.05, 'ver_offset': 0.05, 'required_force': None}]

        # Subscribers
        self.state_sub = rp.Subscriber(
            '/drone_state', DroneStateMarker, self.stateCallback, queue_size=1)
        self.rc_sub = rp.Subscriber(
            '/mavros/rc/in', RCIn, self.rcCallback, queue_size=1)

        # Publishers
        self.trajectory_pub = rp.Publisher(
            '/drone_trajectory', Trajectory, queue_size=1, latch=True)
        self.trajectory_log_pub = rp.Publisher(
            '/drone_trajectory_log', Trajectory, queue_size=1)
        self.mission_state_pub = rp.Publisher(
            '/mission_state', MissionState, queue_size=1)
        self.deployed_setpoint_pub = rp.Publisher(
            '/deployed_setpoint', Vector3, queue_size=1, latch=True)

        rp.loginfo('Disengaging drone magnet')
        self.drone_magnet.droneMagnetDisengage()

        t = threading.Thread(target=self.missionControl())
        t.start()

        rp.spin()

    """
       Callbacks
    """

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

                self.calculateMissionSetpoints()  # (self.H_world_marker)
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

                if np.dot(d_p, d_p) > 0.001 or abs(d_o) > 0.075:
                    rp.logwarn(
                        'The marker\'s position changed too much. Updating setpoints')

                    self.calculateMissionSetpoints()  # (self.H_world_marker)

    def rcCallback(self, msg):
        if self.mission_bttn != msg.channels[9]:
            self.mission_bttn = msg.channels[9]
            self.in_mission = True

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

            self.mission_setpoints[i]['set_x'] = H_world_setpoint[0, 3]
            self.mission_setpoints[i]['set_y'] = H_world_setpoint[1, 3]
            self.mission_setpoints[i]['set_z'] = H_world_setpoint[2, 3]
            self.mission_setpoints[i]['set_o'] = np.arctan2(
                H_world_setpoint[1, 0], H_world_setpoint[0, 0])

        # Make sure the updated setpoints are published
        self.publish_setpoint = True

    def sendSetpoint(self,):
        setpoint_msg = Setpoint()
        setpoint_msg.position.x = self.mission_setpoints[self.mission_step]['set_x']
        setpoint_msg.position.y = self.mission_setpoints[self.mission_step]['set_y']
        setpoint_msg.position.z = self.mission_setpoints[self.mission_step]['set_z']
        setpoint_msg.velocity.x = 0.0
        setpoint_msg.velocity.y = 0.0
        setpoint_msg.velocity.z = 0.0
        setpoint_msg.orientation.x = 0.0
        setpoint_msg.orientation.y = 0.0
        setpoint_msg.orientation.z = self.mission_setpoints[self.mission_step]['set_o']

        trajectory_msg = Trajectory()
        trajectory_msg.header.stamp = rp.Time.now()
        trajectory_msg.trajectory.append(setpoint_msg)

        self.trajectory_pub.publish(trajectory_msg)

    def publishTrajectoryLog(self,):
        setpoint_msg = Setpoint()
        setpoint_msg.position.x = self.mission_setpoints[self.mission_step]['set_x']
        setpoint_msg.position.y = self.mission_setpoints[self.mission_step]['set_y']
        setpoint_msg.position.z = self.mission_setpoints[self.mission_step]['set_z']
        setpoint_msg.velocity.x = 0.0
        setpoint_msg.velocity.y = 0.0
        setpoint_msg.velocity.z = 0.0
        setpoint_msg.orientation.x = 0.0
        setpoint_msg.orientation.y = 0.0
        setpoint_msg.orientation.z = self.mission_setpoints[self.mission_step]['set_o']

        trajectory_msg = Trajectory()
        trajectory_msg.header.stamp = rp.Time.now()
        trajectory_msg.trajectory.append(setpoint_msg)

        self.trajectory_log_pub.publish(trajectory_msg)

    def sendMissionState(self,):
        mission_state_msg = MissionState()
        mission_state_msg.header.stamp = rp.Time.now()
        mission_state_msg.in_contact.data = self.in_contact
        mission_state_msg.wt_sensor.data = self.wt_sensor
        self.mission_state_pub.publish(mission_state_msg)

    def getOffsets(self,):
        dx = self.drone_position[0] - \
            self.mission_setpoints[self.mission_step]['set_x']
        dy = self.drone_position[1] - \
            self.mission_setpoints[self.mission_step]['set_y']
        dz = self.drone_position[2] - \
            self.mission_setpoints[self.mission_step]['set_z']
        do = self.drone_orientation - \
            self.mission_setpoints[self.mission_step]['set_o']

        return dx, dy, dz, do

    def checkPoseCondition(self, dx, dy, dz, do):
        pose_condition = \
            abs(dx) < self.mission_setpoints[self.mission_step]['hor_offset'] and \
            abs(dy) < self.mission_setpoints[self.mission_step]['hor_offset'] and \
            abs(dz) < self.mission_setpoints[self.mission_step]['ver_offset'] and \
            abs(do) < 0.1

        return pose_condition

    def checkDisturbanceCondition(self,):
        if self.mission_setpoints[self.mission_step]['required_force'] < 0.0:
            return self.disturbances[2] < self.mission_setpoints[self.mission_step]['required_force']
        else:
            return self.disturbances[2] > self.mission_setpoints[self.mission_step]['required_force']

    """
       State Machine main function
    """

    def stateMachine(self,):
        # Publish setpoint
        if self.publish_setpoint:
            self.sendSetpoint()
            self.publish_setpoint = False
            self.mission_start_t = rp.Time.now()

        # Check all required conditions for each mission point

        # Approach structure
        if self.mission_step == 0:
            # Check position
            dx, dy, dz, do = self.getOffsets()
            pose_condition = self.checkPoseCondition(dx, dy, dz, do)

            if pose_condition:
                rp.loginfo('Approached structure. Moving to deploy sensor')
                self.mission_step += 1
                self.publish_setpoint = True

        # Try to align, press against base of sensorpack
        elif self.mission_step == 1:
            dt = rp.Time.now() - self.mission_start_t
            dx, dy, dz, do = self.getOffsets()

            pose_condition = self.checkPoseCondition(dx, dy, dz, do)
            dist_condition = self.checkDisturbanceCondition()

            # Check position and required force
            if pose_condition and dist_condition:
                self.in_contact = True
                rp.loginfo('Engaging drone magnet')
                self.drone_magnet.droneMagnetEngage()
                rp.loginfo('Moving to check if the sensor is attached')
                self.mission_step += 1
                self.publish_setpoint = True

            # Check time passed
            elif dt.secs > 20:
                rp.logwarn(
                    'More than 20 seconds have passed since started trying to deploy. Move back and try again')
                self.mission_step -= 1
                self.publish_setpoint = True

            # Check if vertical position too close to setpoint
            elif dz > -0.2:
                rp.logwarn(
                    'Drone is closer than it should be. Move back and try again')
                self.mission_step -= 1
                self.publish_setpoint = True

        # Check if sensor is attached to the drone
        elif self.mission_step == 2:
            dt = rp.Time.now() - self.mission_start_t
            dx, dy, dz, do = self.getOffsets()

            pose_condition = self.checkPoseCondition(dx, dy, dz, do)
            dist_condition = self.checkDisturbanceCondition()

            # Check position and required force
            if pose_condition and dist_condition:
                rp.loginfo('Disengaging sensor magnet')
                self.sensor_magnet_off.switchMagnet()
                rp.loginfo('Sensor retrieved')
                rp.loginfo('Moving away from the structure')
                self.in_contact = False
                self.wt_sensor = True
                self.mission_step += 1
                self.publish_setpoint = True

            # Check time passed
            elif dt.secs > 20:
                rp.logwarn(
                    'More than 20 seconds have passed since started trying to Retrive. Move back and try again')
                rp.loginfo('Disengaging drone magnet')
                self.drone_magnet.droneMagnetDisengage()
                self.in_contact = False
                self.mission_step -= 2
                self.publish_setpoint = True

            # Check if vertical position too close to setpoint or horizontal position too far from setpoint
            elif dz > -0.2 or dx > 0.1 or dy > 0.1:
                rp.logwarn(
                    'Drone\'s position is problematic. Move back and try again')
                rp.logwarn(
                    'dx: {:.3f}, dy: {:.3f}, dz: {:.3f}'.format(dx, dy, dz))
                rp.loginfo('Disengaging Drone magnet')
                self.drone_magnet.droneMagnetDisengage()
                self.in_contact = False
                self.mission_step -= 2
                self.publish_setpoint = True

        # Move away from retrieval position
        elif self.mission_step == 3:
            # Check position
            dx, dy, dz, do = self.getOffsets()
            pose_condition = self.checkPoseCondition(dx, dy, dz, do)

            if pose_condition:
                rp.loginfo('Retrieve mission finished')
                self.in_mission = False

    """
       Main mission function
    """

    def missionControl(self,):
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            # Publish mission state
            self.sendMissionState()

            # Run state machine
            if self.in_mission and self.setpoints_initialized:
                self.publishTrajectoryLog()
                self.stateMachine()

            r.sleep()


if __name__ == '__main__':
    StateMachineNode(5)
