#!/usr/bin/python
import rospy as rp
import numpy as np
import quaternion
import time
from Mag_Eng_DisEng_fns import Mag
from mavros_msgs.msg import RCIn
import threading

from px4_control_msgs.msg import DroneStateMarker, Trajectory, Setpoint

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
        #rp.wait_for_service('/enable_controller')
        rp.loginfo('NMPC is up')

        #Define variables
        self.traj = Trajectory()
        self.Drone_position = [0 , 0, 0]
        self.Drone_distrubance = [0, 0, 0]
        self.mission_bttn = 0
            #Setpoint1
        self.set1_x = -1.50
        self.set1_y = 0.07
        self.set1_z = 1.5
            #Setpoint2
        self.set2_x = -1.50
        self.set2_y = 0.07
        self.set2_z = 2.00
        self.Dist_1 = -0.5
            #Setpoint3
        self.set3_x = -1.50
        self.set3_y = 0.07
        self.set3_z = 1.98
        self.Dist_2 = +0.5
            #Setpoint4(Home)
        self.set4_x = -0.8
        self.set4_y = 0.07
        self.set4_z = 0.26
            #Setpoint5
        self.set5_x = -1.50
        self.set5_y = 0.07
        self.set5_z = 1.98
        self.Dist_3 = -0.5
            #Setpoint6
        self.set6_x = -1.50
        self.set6_y = 0.07
        self.set6_z = 1.98
        self.Dist_4 = +0.5
            #Setpoint7(Home)
        self.set7_x = -0.8
        self.set7_y = 0.07
        self.set7_z = 0.26


        # Subscribers
        self.state_sub = rp.Subscriber(
            '/drone_state', DroneStateMarker, self.stateCallback, queue_size=1)
        self.rc_sub = rp.Subscriber('/mavros/rc/in', RCIn, self.rcCallback, queue_size=1)  
       

        # Publishers
        self.trajectory_pub = rp.Publisher(
            '/drone_trajectory', Trajectory, queue_size=1, latch=True)
        
        t = threading.Thread(target=self.send_mission)
        t.start()

        rp.spin()

    def rcCallback(self, msg):
        self.mission_bttn = msg.channels['9#']     #Assign a button
        

    def stateCallback(self, msg):
        # Get Drone in world position
        self.Drone_position = np.array([msg.pose.position.x,
                                        msg.pose.position.y,
                                        msg.pose.position.z])

        self.Drone_distrubance = np.array([msg.distrubances.x,
                                           msg.distrubances.y,
                                           msg.distrubances.z])

        Drone_att = np.quaternion(msg.pose.orientation.w,
                                  msg.pose.orientation.x,
                                  msg.pose.orientation.y,
                                  msg.pose.orientation.z).normalized()

        H_world_Drone = np.identity(4)
        H_world_Drone[0, 3] = msg.pose.position.x
        H_world_Drone[1, 3] = msg.pose.position.y
        H_world_Drone[2, 3] = msg.pose.position.z
        H_world_Drone[0:3, 0:3] = quaternion.as_rotation_matrix(
                   Drone_att)

        self.Drone_orientation = np.arctan2(
                 H_world_Drone[1, 0], H_world_Drone[0, 0])

        

    # def TrajectoryCallback(self, setpoint_msg):
    #     # Get Drone in world position
    #     self.Setpoint_position = np.array([setpoint_msg.position.x,
    #                                        setpoint_msg.position.y,
    #                                        setpoint_msg.position.z,])

    #     self.Setpoint_orientation = np.array([setpoint_msg.orientation.x,
    #                                           setpoint_msg.orientation.y,
    #                                           setpoint_msg.orientation.z,])
    
    def checkState(self, current_p, desired_p):
        dx = abs(current_p[0] - desired_p[0])
        dy = abs(current_p[1] - desired_p[1])
        dz = abs(current_p[2] - desired_p[2])

        return [dx, dy, dz]

    
    '''
        State Machine Functions
    '''
    def stateMachine(self, Drone_position, Drone_distrubance):

        #Setnew Setpoint        
        setpoint_msg = Setpoint()
        setpoint_msg.position.x = self.set1_x
        setpoint_msg.position.y = self.set1_y
        setpoint_msg.position.z = self.set1_z 
        self.traj.header.stamp = rp.Time.now()
        self.traj.trajectory.append(setpoint_msg)
        self.trajectory_pub.publish(self.traj)
        rp.loginfo('Roughly reached Setpoint-1')


        time.sleep(5)

        if self.mission_bttn == 1494.0: #(Deploy) ##Check 3 way value from the channel after assigning a number
            #Setnew Setpoint        
            setpoint_msg = Setpoint()
            setpoint_msg.position.x = self.set1_x
            setpoint_msg.position.y = self.set1_y
            setpoint_msg.position.z = self.set1_z  
            self.traj.header.stamp = rp.Time.now()
            self.traj.trajectory.append(setpoint_msg)
            self.trajectory_pub.publish(self.traj)
            rp.loginfo('Reaching Setpoint-2')

            x_des = self.set2_x             #Setpoint-2
            y_des = self.set2_y
            z_des = self.set2_z

            check = self.checkState([self.Drone_position[0], self.Drone_position[1], self.Drone_position[2]], 
                                    [x_des, y_des, z_des])

            if (check[0] < 0.05) & (check[1] < 0.05) & (check[2] < 0.02):
                if (Drone_distrubance[2] < self.Dist_1):  #< since its a -ve number (-D_z)
                    Mag.engage(Sensor_Mag)
                    rp.loginfo('Sensor_Magnet engaged at %f, %f, %f', self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                    #Setnew Setpoint
                    # Prepare setpoint message and publish it
                    setpoint_msg = Setpoint()
                    setpoint_msg.position.x = self.set2_x
                    setpoint_msg.position.y = self.set2_y
                    setpoint_msg.position.z = self.Drone_position[2] - 0.02   #Pull 2cm lower
                    self.traj.header.stamp = rp.Time.now()
                    self.traj.trajectory.append(setpoint_msg)
                    self.trajectory_pub.publish(self.traj)
                    rp.loginfo('NewSetpoint-3 sent')

                    x_des = self.set3_x
                    y_des = self.set3_y
                    z_des = self.set3_z

                    check = self.checkState([self.Drone_position[0], self.Drone_position[1], self.Drone_position[2]], 
                                    [x_des, y_des, z_des])

                    if (check[0] < 0.05) & (check[1] < 0.05) & (check[2] < 0.02):
                        if (Drone_distrubance[2] > self.Dist_2):     #(+D_z)
                            #Sensor Co-ods
                            self.sens_x = self.Drone_position[0]
                            self.sens_y = self.Drone_position[1]
                            self.sens_z = self.Drone_position[2]
                            Mag.disengage(Drone_Mag)
                            rp.loginfo('Drone_Magnet Disengaged at %f, %f, %f', self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])
                            #Setnew Setpoint to return Home
                            # Prepare setpoint message and publish it
                            setpoint_msg = Setpoint()
                            setpoint_msg.position.x = -0.8
                            setpoint_msg.position.y = 0.07
                            setpoint_msg.position.z = 0.26
                            self.traj.header.stamp = rp.Time.now()
                            self.traj.trajectory.append(setpoint_msg)
                            self.trajectory_pub.publish(self.traj)
                            rp.loginfo('NewSetpoint-4 sent Returning Home')

                        else:
                            #Setnew Setpoint-2 again
                            # Prepare setpoint message and publish it
                            setpoint_msg = Setpoint()
                            setpoint_msg.position.x = self.set2_x
                            setpoint_msg.position.y = self.set2_y
                            setpoint_msg.position.z = self.set2_z
                            self.traj.header.stamp = rp.Time.now()
                            self.traj.trajectory.append(setpoint_msg)
                            self.trajectory_pub.publish(self.traj)
                            rp.loginfo('Retrying Setpoint2')
                            

                    else: 
                        #Realign to the Setpoint-3
                        #Setnew Setpoint
                        # Prepare setpoint message and publish it
                        setpoint_msg = Setpoint()
                        setpoint_msg.position.x = self.set2_x
                        setpoint_msg.position.y = self.set2_y
                        setpoint_msg.position.z = self.Drone_position[2] - 0.02   #(Pull 2cm lower)
                        self.traj.header.stamp = rp.Time.now()
                        self.traj.trajectory.append(setpoint_msg)
                        self.trajectory_pub.publish(self.traj)
                        rp.loginfo('Realigning to Setpoint-3 ')
                    

                else: 
                    #Realign to the Setpoint-2
                    rp.loginfo("Still Trying to reach Setpoint-2")
                    return[]
            else: 
                #Realign to the Setpoint-2
                rp.loginfo("Realign to Setpoint-2")
                # Prepare setpoint message and publish it
                setpoint_msg = Setpoint()
                setpoint_msg.position.x = self.set2_x
                setpoint_msg.position.y = self.set2_y
                setpoint_msg.position.z = self.set2_z
                self.traj.header.stamp = rp.Time.now()
                self.traj.trajectory.append(setpoint_msg)
                self.trajectory_pub.publish(self.traj)
                rp.loginfo('Retrying Setpoint2')



##************************************************************
        elif self.mission_bttn == 2006:   #(Retrive)          #Check the 3way value from the channel

            #Setnew Setpoint        
            setpoint_msg = Setpoint()
            setpoint_msg.position.x = self.set5_x
            setpoint_msg.position.y = self.set5_y
            setpoint_msg.position.z = self.set5_z 
            self.traj.header.stamp = rp.Time.now()
            self.traj.trajectory.append(setpoint_msg)
            self.trajectory_pub.publish(self.traj)
            rp.loginfo('Reaching Setpoint-5')
                                 
            x_des = self.set5_x             #Setpoint-5
            y_des = self.set5_y
            z_des = self.set5_z      #/self.sens_z

            check = self.checkState([self.Drone_position[0], self.Drone_position[1], self.Drone_position[2]], 
                                    [x_des, y_des, z_des])

            if (check[0] < 0.05) & (check[1] < 0.05) & (check[2] < 0.02):
                if (Drone_distrubance[2] < self.Dist_3):                    
                    Mag.engage(Drone_Mag)
                    rp.loginfo('Drone_Magnet engaged at %f, %f, %f', self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])

                    #Setnew Setpoint
                    # Prepare setpoint message and publish it
                    setpoint_msg = Setpoint()
                    setpoint_msg.position.x = self.set5_x
                    setpoint_msg.position.y = self.set5_y
                    setpoint_msg.position.z = self.Drone_position[0] - 0.02     #Pulling 2cm down
                    self.traj.header.stamp = rp.Time.now()
                    self.traj.trajectory.append(setpoint_msg)
                    self.trajectory_pub.publish(self.traj)
                    rp.loginfo('NewSetpoint-6 sent')

                    x_des = self.set6_x
                    y_des = self.set6_y
                    z_des = self.set6_z

                    check = self.checkState([self.Drone_position[0], self.Drone_position[1], self.Drone_position[2]], 
                                    [x_des, y_des, z_des])

                    if (check[0] < 0.02) & (check[1] < 0.02) & (check[2] < 0.02):
                        if (Drone_distrubance[2] > self.Dist_4):
                            Mag.disengage(Sensor_Mag)
                            rp.loginfo('Sensor_Magnet Disengaged at %f, %f, %f', self.Drone_position[0], self.Drone_position[1], self.Drone_position[2])
                            #Setnew Setpoint to return Home
                            # Prepare setpoint message and publish it
                            setpoint_msg = Setpoint()
                            setpoint_msg.position.x = -0.8
                            setpoint_msg.position.y = 0.07
                            setpoint_msg.position.z = 0.26
                            self.traj.header.stamp = rp.Time.now()
                            self.traj.trajectory.append(setpoint_msg)
                            self.trajectory_pub.publish(self.traj)
                            rp.loginfo('NewSetpoint-7 sent Returning Home')

                        else:                            
                            #Retry Setpoint-5 again     #Engage Sensor Magnet Properly 
                            setpoint_msg = Setpoint()
                            setpoint_msg.position.x = self.set5_x
                            setpoint_msg.position.y = self.set5_y
                            setpoint_msg.position.z = self.set5_z
                            self.traj.header.stamp = rp.Time.now()
                            self.traj.trajectory.append(setpoint_msg)
                            self.trajectory_pub.publish(self.traj)
                            rp.loginfo('Retrying Setpoint5')

                    else: 
                        #Realign to the Setpoint-6                        
                        setpoint_msg = Setpoint()
                        setpoint_msg.position.x = self.set6_x
                        setpoint_msg.position.y = self.set6_y
                        setpoint_msg.position.z = self.Drone_position[2] - 0.02   #(Pull 2cm lower)
                        self.traj.header.stamp = rp.Time.now()
                        self.traj.trajectory.append(setpoint_msg)
                        self.trajectory_pub.publish(self.traj)
                        rp.loginfo('Realigning to Setpoint-6 ')
                else: 
                    #Realign to the Setpoint-5
                    rp.loginfo("Still Trying to reach Setpoint-5")
                    return[0]
            else: 
                #Realign to the Setpoint-5
                setpoint_msg = Setpoint()
                setpoint_msg.position.x = self.set5_x
                setpoint_msg.position.y = self.set5_y
                setpoint_msg.position.z = self.set5_z
                self.traj.header.stamp = rp.Time.now()
                self.traj.trajectory.append(setpoint_msg)
                self.trajectory_pub.publish(self.traj)
                rp.loginfo('Retrying Setpoint5')
##**********************

    def send_mission(self,):
        rp.loginfo('Mission Started')
        r = rp.Rate(self.rate)
        while not rp.is_shutdown():
            self.stateMachine(self.Drone_position, self.Drone_distrubance)
            r.sleep()


if __name__ == '__main__':
    StateMachineNode(2)
        
