#!/usr/bin/env python
## coding: UTF-8

import rospy
from std_msgs.msg import Float32

rospy.init_node("current_controller")
pub = rospy.Publisher("current_controller", Float32, queue_size=10)
rate = rospy.Rate(10)
print("current_stated")

nc = 0.0
dc = 0.05
inc = True

while not rospy.is_shutdown():
  curr = Float32()
  if inc:
    nc = nc + dc
    if nc >= 3.0:
      inc = False
  else:
    nc = nc - dc
    if nc <= -3.0:
      inc = True

  curr.data = nc
  pub.publish(curr)
  rate.sleep()

