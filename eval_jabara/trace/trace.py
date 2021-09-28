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

import pickle
import numpy as np

import matplotlib.pyplot as plt


from stage_py2 import StageControl

axis = "THETA_STAGE"

potangle = 0
mz = 0.0

mzlist = []
predlist = []
timelist = []

cont = True

with open("/home/ubuntu/workspace/src/fluid_network_synergyhand/eval_jabara/trace/weights0827_py2", "rb") as f:
    w = pickle.load(f)

fig, ax = plt.subplots(1, 1)

def forward_propagation(x):
    def norm(x):
        return (x - w[0]) / np.sqrt(w[1])
    def norm_restore(x):
        return x * np.sqrt(w[1]) + w[0]
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


def handler(signal, frame):
  global cont
  cont = False

def potcb(message):
  global potangle
  potangle = message.data

def ftscb(message):
  global mz
  mz = message.data[-1]
  if abs(mz) >= 0.30:
    print "z-moment exceeded the safe value"
    rospy.signal_shutdown("finish")
    motor.setTorque(1024, 2)

subpot = rospy.Subscriber("publish_pot_angle", Int16, potcb)
subfts = rospy.Subscriber("ftsensor_raw", Float32MultiArray, ftscb)

servo_s = [0, 0, 1] # extract, inject, stop
def rotate_servo(value):
  global servo_s
  if value == 0: # stop
    motor.setTorque(1024, 2)
    servo_s = [0, 0, 1]
  elif value > 0: # injection
    motor.setTorque(1024 + value, 2)
    servo_s = [0, 1, 0]
  elif value < 0: # extraction
    motor.setTorque(-value, 2) 
    servo_s = [1, 0, 0]
 
def step_diff(goal, diff_type="servo"):
  global potangle
  cp = 0 # current position
  diff = 0 # difference
  allowable_err = 0 

  if diff_type == "servo":
    cp = motor.getPosition(2)
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
    rotate_servo(-2)
    isdone = False
  elif diff < 0:
    print "inject"
    rotate_servo(3)
    isdone = False
  return isdone  

def servo_pos_control(goal, diff_type="servo"):
  while not step_diff(goal, diff_type=diff_type):
    time.sleep(0.01)


def loop(pot_min):
  global potangle, mz, servo_s, mzlist, predlist, timelist
  # min angle : 6600 pulses of the servo, 45mL in the syringe
  # max angle : -3151 pulses, 
  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract
  servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara
 
  init_mz = mz
  init_pot = potangle
  init_time = time.time()
  init_srv = motor.getPosition(2)

  for goal in [init_srv-8000, init_srv, init_srv-8000, init_srv]:
    i = 0
    while not step_diff(goal, diff_type="servo"):
      cp = motor.getPosition(2)

      t_srv = motor.getPosition(2)
      if t_srv >= 50000:
         t_srv = t_srv - 65536
      pred =  forward_propagation([[t_srv-init_srv, potangle-init_pot]+servo_s])

      print [t_srv-init_srv, potangle-init_pot] + servo_s

      print "{}th pot angle : {}".format(i, potangle-init_pot)
      print "{}th servo angle : {}".format(i, t_srv-init_srv)
      print "{}th mz : {:.5f} [Nm]".format(i, mz-init_mz)
      print "{}th pred : {:.5f} [Nm]".format(i, pred[0])
      print "{}th err : {:.5f} [Nm]".format(i, mz-init_mz - pred[0]) 

      # graph

      if len(mzlist) >= 300:
        mzlist.pop(0)
        predlist.pop(0)
        timelist.pop(0)

      mzlist.append(mz-init_mz)
      predlist.append(pred[0])
      timelist.append(time.time() - init_time)

      ax.set_xlim((timelist[0], timelist[-1]))
      ax.set_ylim((np.min([mzlist, predlist]), np.max([mzlist, predlist])))
  
      mzline, = ax.plot(timelist, mzlist, color="r")
      predline, = ax.plot(timelist, predlist, color="b")

      plt.legend(["true", "pred"])
      plt.pause(0.01)
      
      mzline.remove()
      predline.remove()

      i = i + 1
      time.sleep(0.03)

if __name__ == "__main__":
  
  rospy.init_node("evaluate", disable_signals=True)
  motor = MX64Control(b"/dev/ttyS0", 57600, [2])
  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)

  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract

  try:
    loop(pot_min=268)
  except KeyboardInterrupt:
    print "interrupted"
    rospy.signal_shutdown("finish")
