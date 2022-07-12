import time
import rospy as rp

from px4_control_msgs.msg import Setpoint, Trajectory

# Start a node
rp.init_node('setpoint_node')

# Setup Publisher
trajectory_pub = rp.Publisher('/drone_trajectory', Trajectory, queue_size=1, latch=True)

# Prepare setpoint message
setpoint_msg = Setpoint()
setpoint_msg.position.x = -1.6
setpoint_msg.position.y = 0.0
setpoint_msg.position.z = 1.9
setpoint_msg.velocity.x = 0.0
setpoint_msg.velocity.y = 0.0
setpoint_msg.velocity.z = 0.0
setpoint_msg.orientation.x = 0.0
setpoint_msg.orientation.y = 0.0
setpoint_msg.orientation.z = 0.0

traj = Trajectory()
traj.header.stamp = rp.Time.now()
traj.trajectory.append(setpoint_msg)

# Publish setpoint
trajectory_pub.publish(traj)
time.sleep(1.0)
rp.loginfo('Setpoint sent')
