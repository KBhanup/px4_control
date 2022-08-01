import rospy as rp
import math
import time

from px4_control_msgs.msg import Setpoint, Trajectory

# Create an ascending spiral trajectory
t_traj = 30.0
control_rate = 20.0

radius = 1.0
start_alt = 1.0
final_alt = 2.0
max_yaw = 0.75 * math.pi

steps = int(t_traj * control_rate)

z_velocity = (final_alt - start_alt) / t_traj

trajectory = Trajectory()

for i in range(steps):
    theta_i = (2 * i * math.pi) / steps

    ctheta = math.cos(theta_i)
    stheta = math.sin(theta_i)

    setpoint = Setpoint()

    setpoint.position.x = radius * ctheta
    setpoint.position.y = radius * stheta
    setpoint.position.z = start_alt + z_velocity * i / 20.0

    setpoint.velocity.x = radius * stheta
    setpoint.velocity.y = radius * ctheta
    setpoint.velocity.z = z_velocity

    setpoint.orientation.x = 0.0
    setpoint.orientation.y = 0.0
    setpoint.orientation.z = stheta * max_yaw

    trajectory.trajectory.append(setpoint)

# Add last point with zero velocities
setpoint = Setpoint()

setpoint.position.x = 1.0
setpoint.position.y = 0.0
setpoint.position.z = 2.0

setpoint.velocity.x = 0.0
setpoint.velocity.y = 0.0
setpoint.velocity.z = 0.0

setpoint.orientation.x = 0.0
setpoint.orientation.y = 0.0
setpoint.orientation.z = 0.0

trajectory.trajectory.append(setpoint)

# Publish trajectory
rp.init_node('trajectory_publisher')
trajectory.header.stamp = rp.Time.now()
pub = rp.Publisher("/drone_trajectory", Trajectory, queue_size=1, latch=True)
pub.publish(trajectory)
time.sleep(1.0)
