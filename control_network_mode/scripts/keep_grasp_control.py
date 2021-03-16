#!/usr/bin/env python
## coding: UTF-8

import time
import rospy
from std_msgs.msg import Int16MultiArray, Int8, Int16, Float32

target_pot = [0, 2, 3, 4]
modetopot = [3, 0, 1, 3, 3, 3, 5, 3]
target_position = 40
angles = [0, 0, 0, 0, 0, 0, 0]

curr = 0
pre_pot = [0, 0, 0, 0, 0, 0, 0]
detect_grasp = 0
is_grasped = False

dc = 0.03

def GetPotCb(message):
  global curr, is_grasped, pre_pot, target_pot, detect_grasp

  error = 0
  if not is_grasped:
    for i in target_pot:
      error = error + abs(pre_pot[i] - message.data[i])
      pre_pot[i] = message.data[i]
    
    if error < 2 and detect_grasp > 10:
      is_grasped = True
      curr = curr + 0.4
    elif error < 2:
      detect_grasp = detect_grasp + 1
      curr = curr + 0.03
    else:
      detect_grasp = 0
      curr = curr + 0.03

  send = Float32()
  send.data = curr

  rospy.loginfo("error:{}, current:{}, is_grasped:{}".format(error, curr, is_grasped))
  if(abs(send.data) >= 3.5):
    rospy.logwarn("!!!current_exceeded:{} [A]".format(send.data))
    send.data = 0
    pub.publish(send)
    rospy.signal_shutdown("finish")

  pub.publish(send)


def TargetPositionCb(message):
  global target_position
  target_position = message.data
  
rospy.init_node("position_controller")

sub2 = rospy.Subscriber("send_pot", Int16MultiArray, GetPotCb)
sub3 = rospy.Subscriber("target_angle", Int16, TargetPositionCb)
pub = rospy.Publisher("current_controller", Float32, queue_size=10)

rospy.spin()
