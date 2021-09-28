#!/usr/bin/env python
## coding: UTF-8

import rospy
import time
import signal
import sys
from serial_dx_test.dx_comm import MX64Control
from std_msgs.msg import Float32MultiArray, Int16

from stage_py2 import StageControl

axis = "THETA_STAGE"


potangle = 0
mz = 0.0

cont = True

def handler(signal, frame):
  global cont
  cont = False

def potcb(message):
  global potangle
  potangle = message.data

def ftscb(message):
  global mz
  mz = message.data[-1]

subpot = rospy.Subscriber("publish_pot_angle", Int16, potcb)
subfts = rospy.Subscriber("ftsensor_raw", Float32MultiArray, ftscb)

if __name__ == "__main__":
  
  rospy.init_node("evaluate", disable_signals=True)
  motor = MX64Control(b"/dev/ttyS0", 57600, [1])

  print "current pot angle : {}".format(potangle)
  print "current servo angle : {}".format(motor.getPosition(1))
  print "current mz : {:.5f} [Nm]".format(mz)

