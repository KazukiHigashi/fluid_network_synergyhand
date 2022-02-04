#!/usr/bin/env python
## coding: UTF-8

import time
import sys
import os
from serial_dx_test.dx_comm import MX64Control

import pickle
import numpy as np

class Predictor:
  def __init__(self, weights_path):
    self.w = None
    with open(weights_path, "rb") as f:
      self.w = pickle.load(f)
    
  def forward_propagation(self, x):
    def norm(x):
        return (x - self.w[0]) / np.sqrt(self.w[1])
    def relu(x):
        return np.maximum(0, x)
    W12 = self.w[3]
    b1 = self.w[4]
    W23 = self.w[5]
    b2 = self.w[6]
    W34 = self.w[7]
    b3 = self.w[8]
    
    nx = norm(x)

    if len(nx) == 1:
        u = np.dot(W34.T, relu(np.dot(W23.T, relu(np.dot(W12.T, nx[0])+b1))+b2))+b3
    else:
        u = [np.dot(W34.T, relu(np.dot(W23.T, relu(np.dot(W12.T, nxx)+b1))+b2))+b3 for nxx in nx]
    return np.array(u)

 
class MotorController:
  def __init__(self, control_target):
    self.motor = MX64Control(b"/dev/ttyS0", 57600, [control_target])
    self.motor_s = [0, 0, 1] # [extract, inject, stop]
        
  def rotate_servo(self, value):
    global servo_s
    if value == 0: # stop
      self.motor.setTorque(1024, target)
      self.motor_s = [0, 0, 1]
    elif value > 0: # injection
      self.motor.setTorque(1024 + value, target)
      self.motor_s = [0, 1, 0]
    elif value < 0: # extraction
      self.motor.setTorque(-value, target) 
      self.motor_s = [1, 0, 0]
   
  def step_diff(self, goal, diff_type="servo", max_pressure=30, speed=1):
    global potangle, pressure
    cp = 0 # current position
    diff = 0 # difference
    allowable_err = 0 

    if diff_type == "servo":
      cp = self.motor.getPosition(target)
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
      self.rotate_servo(0)
      isdone = True
    elif diff > 0:
      print "extract"
      if pressure < -25:
        print "pressure becomes under -" + str(max_pressure) + "kPa"
        self.rotate_servo(0)
        isdone = True
      else:  
        self.rotate_servo(-3*speed)
        isdone = False
    elif diff < 0:
      print "inject"
      if pressure > max_pressure:
        print "presssure becomes over" + str(max_pressure)+ "kPa"
        self.rotate_servo(0)
        isdone = True
      else:  
        self.rotate_servo(4*speed)
        isdone = False
    return isdone

  def servo_pos_control(self, goal, diff_type="servo"):
    while not self.step_diff(goal, diff_type=diff_type):
      time.sleep(0.01)


