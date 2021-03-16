#!/usr/bin/env python
## coding: UTF-8

import time
import rospy
from std_msgs.msg import Int16MultiArray, Int8, Int16, Float32

rospy.init_node("demo1_powergrasp")

pub1 = rospy.Publisher("mode_setter", Int8, queue_size=10)
pub2 = rospy.Publisher("target_angle", Int16, queue_size=10)

start = time.time()

def publish_all(a,c,d):
  global pub1, pub2  
  mode = Int8()
  target_angle = Int16()
  mode.data = a
  target_angle.data = c

  pub1.publish(mode)
  pub2.publish(target_angle)
  time.sleep(d)

time.sleep(1)

# Init
publish_all(0, 35, 15)  

# Thumb Rotation
publish_all(2, 70, 20)

# Grasp
publish_all(4, 93, 15)

