#!/usr/bin/env python
## coding: UTF-8

import rospy
import time
import signal
import sys
import os
import datetime
import csv
from serial_dx_test.dx_comm import MX64Control
from std_msgs.msg import Float32MultiArray, Int16, Float32

from stage_py2 import StageControl

axis = "THETA_STAGE"

mz = 0.0

cont = True

def handler(signal, frame):
  global cont
  cont = False

def ftscb(message):
  global mz
  mz = message.data[-1]

subfts = rospy.Subscriber("ftsensor_raw", Float32MultiArray, ftscb)

def servo_pos_control(goal):
  while True:
    cp = motor.getPosition(2)
    if cp >= 50000:
      cp = cp - 65536
    diff = goal-cp

    print diff
    if abs(diff) <= 10:
      break
    elif diff > 0:
      print "extract"
      motor.setTorque(2, 2)
    elif diff < 0:
      print "inject"
      motor.setTorque(1027, 2)

    time.sleep(0.01)


if __name__ == "__main__":
  
  rospy.init_node("evaluate", disable_signals=True)
  motor = MX64Control(b"/dev/ttyS0", 57600, [2])
  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)

  try:
    # servo_pos_control(6600)
    t_stage.absolute_move(axis=axis, mm=0, speed=50)
  except KeyboardInterrupt:
    print "interrupted"
    rospy.signal_shutdown("finish")
    motor.setTorque(1024, 2)

