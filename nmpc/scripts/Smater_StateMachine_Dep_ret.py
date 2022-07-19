#!/usr/bin/python
import rospy as rp
from std_msgs.msg import Bool
import numpy as np
import time
from Mag_Eng_DisEng_fns import Mag
from mavros_msgs.msg import RCIn
import threading

from px4_control_msgs.msg import DroneStateMarker, Trajectory, Setpoint


class StateMachineNode():
    """
      Simple state machine that checks the state and when the position and 
      Distrubances meet the criteria either Deploy or Retrive the sensor
      based on request.
    """

    def __init__(self, rate):
        rp.init_node('state_machine_node')
        self.rate = rate

        # Define variables
        self.Sensor_MagON = Mag(14)
        self.Sensor_MagOFF = Mag(15)
        self.Drone_Mag = Mag(4)
        self.SEng_position = [- 1.6, 0.0, 1.99]
        self.Ddiseng_position = [-1.573562, -0.007904, 1.975114]
        self.setpoint_send = False
        self.in_mission = False
        self.in_contact = False
        self.mission_step = 5  # status of Deploymission
        #self.retivemission_step = 0

        self.mission_points = [[-1.6, 0.0, 1.5, 0.02, None],  # desired Setpoints for Deploy Mission
                               [- 1.6, 0.0, 2.11, 0.2, -0.7],
                               [- 1.6, 0.0, 1.90, 0.2, +0.7],
                               [- 1.6, 0.0, 1.3, 0.02, None],
                               [- 0.5, 0.0, 0.26, 0.02, None],
                               ]

        self.retrive_points = [[0, 0.0, 0, 0, None],
                               [0, 0.0, 0, 0, None],
                               [0, 0.0, 0, 0, None],
                               [0, 0.0, 0, 0, None],
                               [0, 0.0, 0, 0, None],
                               # desired Setpoints for Retrive Mission
                               [-1.6, 0.0, 1.5, 0.02, None],
                               [self.Ddiseng_position[0], self.Ddiseng_position[1],
                                   self.Ddiseng_position[2]+0.1, 0.2, -0.7],
                               [self.Ddiseng_position[0], self.Ddiseng_position[1],
                                   self.Ddiseng_position[2]-0.2, 0.2, +0.7],
                               [- 1.6, 0.0, 1.3, 0.02, None],
                               [- 0.5, 0.0, 0.26, 0.02, None],
                               ]

        self.Drone_position = [0, 0, 0]
        self.Drone_disturbance = [0, 0, 0]
        self.mission_bttn = 0

        # Subscribers
        self.state_sub = rp.Subscriber(
            '/drone_state', DroneStateMarker, self.stateCallback, queue_size=1)
        self.rc_sub = rp.Subscriber(
            '/mavros/rc/in', RCIn, self.rcCallback, queue_size=1)

        # Publishers
        self.trajectory_pub = rp.Publisher(
            '/drone_trajectory', Trajectory, queue_size=1, latch=True)
        self.in_contact_pub = rp.Publisher(
            '/mission_state', Bool, queue_size=1)
        # pub to /mission_state  :contact flags
        t = threading.Thread(target=self.send_mission)
        t.start()

        rp.spin()

    def rcCallback(self, msg):
        # Assign a button on RC to set either Deploy(Down2006) or Retrive(Top982) Mission
        if self.mission_bttn != msg.channels[9]:
            self.mission_bttn = msg.channels[9]
            self.in_mission = True

    def stateCallback(self, msg):
        # Get Drone in world position
        self.Drone_position = np.array([msg.pose.position.x,
                                        msg.pose.position.y,
                                        msg.pose.position.z])

        self.Drone_disturbance = np.array([msg.disturbances.x,
                                           msg.disturbances.y,
                                           msg.disturbances.z])

    def checkState(self, missionDR_points):
        dx = abs(self.Drone_position[0] -
                 (missionDR_points[self.mission_step][0]))
        dy = abs(self.Drone_position[1] -
                 (missionDR_points[self.mission_step][1]))
        dz = abs(self.Drone_position[2] -
                 (missionDR_points[self.mission_step][2]))

        dist_z = 2.0
        if (missionDR_points[self.mission_step][4]) is not None:
            dist_z = abs(
                (missionDR_points[self.mission_step][4]) + self.Drone_disturbance[2])

        # 1.7 when estDz=0.8
        if (dx < 0.05) & (dy < 0.05) & (dz < missionDR_points[self.mission_step][3]) & (dist_z > 1.4):
            return True
        else:
            return False

    def send_setpoint(self,):
        # Setnew Setpoint
        if self.mission_bttn == 2006.0:
            setpoint_msg = Setpoint()
            setpoint_msg.position.x = (
                self.mission_points[self.mission_step][0])
            setpoint_msg.position.y = (
                self.mission_points[self.mission_step][1])
            setpoint_msg.position.z = (
                self.mission_points[self.mission_step][2])
            traj = Trajectory()
            traj.header.stamp = rp.Time.now()
            traj.trajectory.append(setpoint_msg)
            self.trajectory_pub.publish(traj)

        elif self.mission_bttn == 982.0:
            setpoint_msg = Setpoint()
            setpoint_msg.position.x = (
                self.retrive_points[self.mission_step][0])
            setpoint_msg.position.y = (
                self.retrive_points[self.mission_step][1])
            setpoint_msg.position.z = (
                self.retrive_points[self.mission_step][2])
            traj = Trajectory()
            traj.header.stamp = rp.Time.now()
            traj.trajectory.append(setpoint_msg)
            self.trajectory_pub.publish(traj)
    '''
        State Machine Functions
    '''

    def stateMachine(self,):
        if(not self.setpoint_send):
            self.send_setpoint()
            self.setpoint_send = True

        if self.mission_bttn == 2006.0:  # Deploy Button(down)
#            rp.loginfo('Mission set to Deploy')
            if self.checkState(self.mission_points):
                rp.loginfo('New Setpoint-%d reached', self.mission_step)

                if(self.mission_step == 1):
                    self.Sensor_MagON.Sn_Magengage()
                    self.in_contact = True                    
                    self.SEng_position[0] = self.Drone_position[0]
                    self.SEng_position[1] = self.Drone_position[1]
                    self.SEng_position[2] = self.Drone_position[2]
                    rp.loginfo('Sensor_Magnet engaged at %f, %f, %f',
                               self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                elif(self.mission_step == 2):
                    self.Drone_Mag.dr_Magdisengage()
                    self.in_contact = False                    
                    self.Ddiseng_position[0] = self.Drone_position[0]
                    self.Ddiseng_position[1] = self.Drone_position[1]
                    self.Ddiseng_position[2] = self.Drone_position[2]                
                    rp.loginfo('Drone_Magnet Disengaged at %f, %f, %f',
                               self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                elif(self.mission_step == 4):
                    rp.loginfo('Deploy_Mission Acomplished!!')
                    self.in_mission = False

                self.mission_step += 1
                self.setpoint_send = False

        elif self.mission_bttn == 982.0:  # Retrive Button(Top)
#            rp.loginfo('Mission switched to Retrivel')
            if self.checkState(self.retrive_points):
                rp.loginfo('New Setpoint-%d reached', self.mission_step)

                if(self.mission_step == 6):
                    self.Drone_Mag.dr_Magengage()
                    self.in_contact = True                    
                    rp.loginfo('Drone_Magnet engaged at %f, %f, %f',
                               self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                elif(self.mission_step == 7):
                    self.Sensor_MagOFF.Sn_Magdisengage()
                    self.in_contact = False                    
                    rp.loginfo('Sensor_Magnet Disengaged at %f, %f, %f',
                               self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                elif(self.mission_step == 9):
                    rp.loginfo('Retrive_Mission Acomplished!!')
                    self.in_mission = False

                self.mission_step += 1
                self.setpoint_send = False

    def send_mission(self,):
        rp.loginfo('Mission Started')
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            self.in_contact_pub.publish(self.in_contact)
            if self.in_mission:
                self.stateMachine()
            r.sleep()


if __name__ == '__main__':
    StateMachineNode(2)