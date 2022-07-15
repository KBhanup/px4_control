#!/usr/bin/python
import rospy as rp
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
        self.Sensor_MagOFF =Mag(15)
        self.Drone_Mag =Mag(4)
        self.setpoint_send = False
        self.mission_step=0                                 #status of mission
        
        self.mission_points = [[-1.6, 0.0, 1.5, 0.02, None],      #desired Setpoints
                               [- 1.6, 0.0, 2.11, 0.2, -0.7],
                               [- 1.6, 0.0, 1.90, 0.2,+0.7],
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

        t = threading.Thread(target=self.send_mission)
        t.start()

        rp.spin()

    def rcCallback(self, msg):
        # Assign a button on RC to set either Deploy(Down2006) or Retrive(Top982) Mission
        self.mission_bttn = msg.channels[9]

    def stateCallback(self, msg):
        # Get Drone in world position
        self.Drone_position = np.array([msg.pose.position.x,
                                        msg.pose.position.y,
                                        msg.pose.position.z])

        self.Drone_disturbance = np.array([msg.disturbances.x,
                                           msg.disturbances.y,
                                           msg.disturbances.z])

    def checkState(self):
        dx = abs(self.Drone_position[0] - (self.mission_points[self.mission_step][0]))
        dy = abs(self.Drone_position[1] - (self.mission_points[self.mission_step][1]))
        dz = abs(self.Drone_position[2] - (self.mission_points[self.mission_step][2]))

        dist_z = 2.0
        if (self.mission_points[self.mission_step][4]) is not None:
            dist_z = abs((self.mission_points[self.mission_step][4]) + self.Drone_disturbance[2])

        if (dx < 0.05) & (dy < 0.05) & (dz < self.mission_points[self.mission_step][3]) & (dist_z > 1.4):        #1.7 when estDz=0.8
            return True
        else: 
            return False

    def send_setpoint(self,):
        # Setnew Setpoint
        setpoint_msg = Setpoint()
        setpoint_msg.position.x = (self.mission_points[self.mission_step][0])
        setpoint_msg.position.y = (self.mission_points[self.mission_step][1])
        setpoint_msg.position.z = (self.mission_points[self.mission_step][2])
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

        if self.mission_bttn == 2006.0:                     #Deploy Button(down)
            if self.checkState():
                rp.loginfo('New Setpoint-%d reached', self.mission_step)

                if(self.mission_step == 1):
                    self.Sensor_MagON.Sn_Magengage()
                    rp.loginfo('Sensor_Magnet engaged at %f, %f, %f', self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                if(self.mission_step == 2):
                    self.Drone_Mag.dr_Magdisengage()
                    rp.loginfo('Drone_Magnet Disengaged at %f, %f, %f', self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                self.mission_step += 1
                self.setpoint_send = False


                if(self.mission_step == 3):
                    rp.loginfo('Deploy_Mission Acomplished!!')
                    self.mission_step =0


        elif self.mission_bttn == 982.0:                    #Retrive Button(Top)
            if self.checkState():
                rp.loginfo('New Setpoint-%d reached', self.mission_step)

                if(self.mission_step == 1):
                    self.Drone_Mag.dr_Magengage()                    
                    rp.loginfo('Drone_Magnet engaged at %f, %f, %f', self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                if(self.mission_step == 2):
                    self.Sensor_MagOFF.Sn_Magdisengage()
                    rp.loginfo('Sensor_Magnet Disengaged at %f, %f, %f', self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                self.mission_step += 1
                self.setpoint_send = False


                if(self.mission_step == 3):
                    rp.loginfo('Retrive_Mission Acomplished!!')
                    self.mission_step =0

    def send_mission(self,):
        rp.loginfo('Mission Started')
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            self.stateMachine()
            r.sleep()


if __name__ == '__main__':
    StateMachineNode(2)
