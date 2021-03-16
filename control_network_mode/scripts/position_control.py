#!/usr/bin/env python
## coding: UTF-8

import time
import rospy
from std_msgs.msg import Int16MultiArray, Int8, Int16, Float32

target_pot = 3
modetopot = [3, 0, 1, 3, 3, 3, 5, 3]
target_position = 40
angles = [0, 0, 0, 0, 0, 0, 0]
Kp = 0.025
Ki = 0.005
Kd = 0.001
preP = None

I = 0

Ts = 0.1

def TargetPotCb(message):
  global target_pot, preP, I
  preP = None
  I = 0
  target_pot = modetopot[message.data]

def GetPotCb(message):
  global preP, Kp, Ki, Kd, Ts, I

  for i in range(7):
    angles[i] = message.data[i]

  P = target_position - angles[target_pot]
  if preP is None:
    preP = P
 
  I =  I + P*Ts
  D = (P - preP)/Ts

  preP = P

  curr = Float32()
  curr.data = Kp*P + Ki*I + Kd*D

  rospy.loginfo("target:{}, pot_angle:{}, current:{}".format(target_position, angles[target_pot],curr.data))
  if(abs(curr.data) >= 3):
    rospy.logwarn("!!!current_exceeded:{} [A]".format(curr.data))
    curr.data = 0
    pub.publish(curr)
    rospy.signal_shutdown("finish")

  pub.publish(curr)


def TargetPositionCb(message):
  global target_position
  target_position = message.data
  
rospy.init_node("position_controller")

sub1 = rospy.Subscriber("mode_setter", Int8, TargetPotCb)
sub2 = rospy.Subscriber("send_pot", Int16MultiArray, GetPotCb)
sub3 = rospy.Subscriber("target_angle", Int16, TargetPositionCb)
pub = rospy.Publisher("current_controller", Float32, queue_size=10)

rospy.spin()
