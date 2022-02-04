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
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Float32, Int16

import pickle
import numpy as np

axis = "THETA_STAGE"

target = 1
potangle = 0

cont = True

def potcb(message):
  global potangle
  potangle = message.data[0]
  pubpot.publish(message.data[0])
  pubbs.publish(message.data[1])
  pubpres.publish(message.data[2])

def ftscb(message):
  global mz
  mz = message.data[-1]
  pubmz.publish(-1000*mz)

subpot = rospy.Subscriber("publish_pot_angle", Int16MultiArray, potcb)
subfts = rospy.Subscriber("ftsensor_raw", Float32MultiArray, ftscb)

servo_s = [0, 0, 1] # extract, inject, stop
def rotate_servo(value):
  global servo_s
  if value == 0: # stop
    motor.setTorque(1024, target)
    servo_s = [0, 0, 1]
  elif value > 0: # injection
    motor.setTorque(1024 + value, target)
    servo_s = [0, 1, 0]
  elif value < 0: # extraction
    motor.setTorque(-value, target) 
    servo_s = [1, 0, 0]
 
def step_diff(goal, diff_type="servo"):
  global potangle
  cp = 0 # current position
  diff = 0 # difference
  allowable_err = 0 

  if diff_type == "servo":
    cp = motor.getPosition(target)
    diff = goal - cp
    allowable_err = 10

  elif diff_type == "pot": # potentiometer
    diff = potangle - goal 
    allowable_err = 1

  print diff
  if abs(diff) <= allowable_err:
    rotate_servo(0)
    isdone = True
  elif diff > 0:
    print "extract"
    rotate_servo(-4)
    isdone = False
  elif diff < 0:
    print "inject"
    rotate_servo(5)
    isdone = False
  return isdone  

def servo_pos_control(goal, diff_type="servo"):
  while not step_diff(goal, diff_type=diff_type):
    time.sleep(0.01)


def loop(pot_min):
  global potangle
  # min angle : 6600 pulses of the servo, 45mL in the syringe
  # max angle : -3151 pulses, 
  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract
  servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara
 
  init_pot = potangle
  init_time = time.time()
  init_srv = motor.getPosition(target)

  for goal in [init_srv-12000, init_srv, init_srv-12000, init_srv]:
    while not step_diff(goal, diff_type="servo"):
      cp = motor.getPosition(target)

      t_srv = motor.getPosition(target)
      time.sleep(0.03)

pubpot = rospy.Publisher("potentiometer", Int16, queue_size=10)
pubbs = rospy.Publisher("bellows_sensor", Int16, queue_size=10)
pubpres = rospy.Publisher("pressure", Int16, queue_size=10)
pubmz = rospy.Publisher("mz", Float32, queue_size=10)

if __name__ == "__main__":
  rospy.init_node("evaluate", disable_signals=True)
  motor = MX64Control(b"/dev/ttyS0", 57600, [target])

  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract

  try:
    loop(pot_min=280)
  except KeyboardInterrupt:
    print "interrupted"
    rospy.signal_shutdown("finish")
