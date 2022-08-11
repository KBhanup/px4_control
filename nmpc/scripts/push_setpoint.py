import time
import rospy as rp

from px4_control_msgs.msg import Setpoint, Trajectory, MissionState

# Start a node
rp.init_node('setpoint_node')

# Setup Publishers
trajectory_pub = rp.Publisher('/drone_trajectory', Trajectory, queue_size=1, latch=True)
mission_state_pub = rp.Publisher('/mission_state', MissionState, queue_size=1, latch=True)

# Prepare setpoint message
setpoint_msg = Setpoint()
setpoint_msg.position.x = 0.0
setpoint_msg.position.y = 0.0
setpoint_msg.position.z = 1.5
setpoint_msg.velocity.x = 0.0
setpoint_msg.velocity.y = 0.0
setpoint_msg.velocity.z = 0.0
setpoint_msg.orientation.x = 0.0
setpoint_msg.orientation.y = 0.0
setpoint_msg.orientation.z = 0.0

traj = Trajectory()
traj.header.stamp = rp.Time.now()
traj.trajectory.append(setpoint_msg)

# Prepare mission state message
mission_state_msg = MissionState()
mission_state_msg.in_contact.data = False
mission_state_msg.wt_sensor.data = False

# Publish mission state
mission_state_pub.publish(mission_state_msg)
time.sleep(1.0)
rp.loginfo('Mission state sent')

# Publish setpoint
trajectory_pub.publish(traj)
time.sleep(1.0)
rp.loginfo('Setpoint sent')
