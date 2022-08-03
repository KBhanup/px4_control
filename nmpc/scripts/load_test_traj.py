import rospy as rp
import math
import time

from px4_control_msgs.msg import Setpoint, Trajectory

t_traj = 20.0
control_rate = 10.0

l = 0.5
alt = 0.5
theta = 0.0
steps = int(t_traj * control_rate)

traj = []

for i in range(steps):
  theta_i = (2 * i * math.pi) / steps

  ctheta = math.cos(theta_i)
  stheta = math.sin(theta_i)

  set_point = Setpoint()

  set_point.position.x = l * ctheta
  set_point.position.y = l * stheta
  set_point.position.z = alt

  set_point.velocity.x = -l * stheta
  set_point.velocity.y = l * ctheta
  set_point.velocity.z = 0.0

  set_point.orientation.x = 0.0
  set_point.orientation.y = 0.0
  set_point.orientation.z = 0.5 * (theta_i - math.pi)

  traj.append(set_point)

# Publish trajectory
rp.init_node('trajectory_publisher')
pub = rp.Publisher("/drone_trajectory", Trajectory, queue_size=1, latch=True)
pub.publish(traj)
time.sleep(1.0)