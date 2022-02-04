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

from stage_py2 import StageControl

axis = "THETA_STAGE"

target = 1
potangle = 0
mz = 0.0
stretch = 0
pressure = 0

MAXSTRETCH = 856.0
MINSTRETCH = 150.0

cont = True

def analog2p(analog):
  # return 1000*(0.011*(analog*0.02444988-5.01222494)-0.1) # [kPa]
  return 1000*(0.011*(analog*0.00611124694+6.7469437653)-0.1) # [kPa]

with open("/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/scripts/weights0113_py2", "rb") as f:
  w = pickle.load(f)

def forward_propagation(x):
    def norm(x):
        return (x - w[0]) / np.sqrt(w[1])
    def relu(x):
        return np.maximum(0, x)
    W12 = w[3]
    b1 = w[4]
    W23 = w[5]
    b2 = w[6]
    W34 = w[7]
    b3 = w[8]
    
    nx = norm(x)

    if len(nx) == 1:
        u = np.dot(W34.T, relu(np.dot(W23.T, relu(np.dot(W12.T, nx[0])+b1))+b2))+b3
    else:
        u = [np.dot(W34.T, relu(np.dot(W23.T, relu(np.dot(W12.T, nxx)+b1))+b2))+b3 for nxx in nx]
    return np.array(u)

def potcb(message):
  global potangle, stretch, pressure
  potangle = message.data[0]
  stretch = message.data[1]
  pressure = analog2p(message.data[2])
  pubpot.publish(potangle)
  pubbs.publish(stretch)
  pubpres.publish(pressure)

def ftscb(message):
  global mz
  mz = message.data[-1]

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
 
def step_diff(goal, diff_type="servo", max_pressure=30):
  global potangle, pressure
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

  elif diff_type == "pressure":
    diff = pressure - goal
    allowable_err = 0.2

  print diff
  if abs(diff) <= allowable_err:
    rotate_servo(0)
    isdone = True
  elif diff > 0:
    print "extract"
    rotate_servo(-9)
    isdone = False
  elif diff < 0:
    print "inject"
    if pressure > max_pressure:
      print "presssure becomes over" + str(max_pressure)+ "kPa"
      rotate_servo(0)
      isdone = True
    else:  
      rotate_servo(12)
      isdone = False
  return isdone  

def servo_pos_control(goal, diff_type="servo"):
  while not step_diff(goal, diff_type=diff_type):
    time.sleep(0.01)


def loop(pot_min, filepath, mmlist=[0, 10], max_pressure_list=[10, 20, 30]):
  global potangle, stretch, mz, pressure, servo_s
  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract
  servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara

  # record all data
  for mm in mmlist:
    t_stage.absolute_move(axis=axis, mm=mm, speed=30)
 
    time.sleep(10)
    print(max_pressure_list)
    for n, max_pressure in enumerate(max_pressure_list):
      servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara
      time.sleep(5)
      init_srv = motor.getPosition(target)
      init_time = time.time()
      init_pot = potangle
      init_mz = mz

      maxpulse = 17000 - mm*150
      for goal, diff_type in [(init_srv-maxpulse, "servo"), (init_srv, "servo")]:
        for i in range(300):
          t_srv = motor.getPosition(target)
          pred = forward_propagation(np.array([[potangle-init_pot, (float(stretch)-MINSTRETCH)/(MAXSTRETCH-MINSTRETCH), pressure]+servo_s]))[0]
          pubpred.publish(-1000*pred)
          pubmz.publish(-1000*(mz-init_mz))

          time.sleep(0.03)

        while not step_diff(goal=goal, diff_type=diff_type, max_pressure=max_pressure):
          t_srv = motor.getPosition(target)
          pred = forward_propagation(np.array([[potangle-init_pot, (float(stretch)-MINSTRETCH)/(MAXSTRETCH-MINSTRETCH), pressure]+servo_s]))[0]
          pubpred.publish(-1000*pred)
          pubmz.publish(-1000*(mz-init_mz))

          time.sleep(0.03)
        
        for i in range(300):
          t_srv = motor.getPosition(target)
          pred = forward_propagation(np.array([[potangle-init_pot, (float(stretch)-MINSTRETCH)/(MAXSTRETCH-MINSTRETCH), pressure]+servo_s]))[0]
          pubpred.publish(-1000*pred)
          pubmz.publish(-1000*(mz-init_mz))

          time.sleep(0.03)
   
pubpot = rospy.Publisher("potentiometer", Int16, queue_size=10)
pubbs = rospy.Publisher("bellows_sensor", Int16, queue_size=10)
pubpres = rospy.Publisher("pressure", Int16, queue_size=10)
pubmz = rospy.Publisher("mz", Float32, queue_size=10)
pubpred = rospy.Publisher("prediction", Float32, queue_size=10)

if __name__ == "__main__":
  rospy.init_node("evaluate", disable_signals=True)
  motor = MX64Control(b"/dev/ttyS0", 57600, [target])
  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)

  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract
  
  mmlist = range(0, 10, 2)
  print(mmlist)

  try:
    loop(pot_min=280, filepath="hoge", mmlist=mmlist, max_pressure_list=[25, 25, 25, 25, 25])
  except KeyboardInterrupt:
    print "interrupted"
    rospy.signal_shutdown("finish")
