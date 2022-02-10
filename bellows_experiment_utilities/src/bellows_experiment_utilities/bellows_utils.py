#!/usr/bin/env python
## coding: UTF-8

import rospy
import time
import sys
import os
from serial_dx_test.dx_comm import MX64Control
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Float32, Int16

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

class PressureSensor:
  def __init__(self, scpmin=-30, scpmax=50, fspmin=-100, fspmax=100):
    # pressure [kPa] to a percentage in the adjusted scale [%] : ax + b = y
    invmatrix = np.linalg.inv(np.array([[fspmin, 1],[fspmax, 1]]))
    ab = np.dot(invmatrix, np.array([[0],[100]]))
    self.a = ab[0][0]
    self.b = ab[1][0]

    print "scaled minimum percentage: " + str(self.pressure2percentage(scpmin)) + "[%]"
    print "scaled maximum percentage: " + str(self.pressure2percentage(scpmax)) + "[%]"

    # analog voltage [V] to pressure [kPa] : cv + d = x
    invmatrix = np.linalg.inv(np.array([[1, 1], [5, 1]]))
    cd = np.dot(invmatrix, np.array([[scpmin], [scpmax]]))
    self.c = cd[0][0]
    self.d = cd[1][0]

  def pressure2percentage(self, x):
    return self.a*float(x) + self.b
  
  def analog2pressure(self, v):
    return self.c*float(v) + self.d

  def digital2pressure(self, z):
    return self.c*(5.0/1023.0)*float(z) + self.d


class DataManager:
  def __init__(self, max_stretch, min_stretch):
    self.subpot = rospy.Subscriber("publish_pot_angle", Int16MultiArray, self.potcb)
    self.subfts = rospy.Subscriber("ftsensor_raw", Float32MultiArray, self.ftscb)

    self.potangle = 0
    self.mz = 0
    self.stretch = 0
    self.normed_stretch = 0
    self.pulses = 0
    self.pressure = 0
    
    self.ps = PressureSensor(-30, 50, -100, 100)

    self.max_stretch = max_stretch
    self.min_stretch = min_stretch

    self.pubpot = rospy.Publisher("potentiometer", Int16, queue_size=10)
    self.pubbs = rospy.Publisher("bellows_sensor", Int16, queue_size=10)
    self.pubpres = rospy.Publisher("pressure", Float32, queue_size=10)
    self.pubmz = rospy.Publisher("mz", Float32, queue_size=10)
    self.pubpred = rospy.Publisher("prediction", Float32, queue_size=10)

  def potcb(self, message):
    self.potangle = message.data[0]
    self.stretch = message.data[1]
    self.normed_stretch = (float(message.data[1]-self.min_stretch)/(self.max_stretch-self.min_stretch))
    self.pressure = self.ps.digital2pressure(message.data[2])

    self.pubpot.publish(self.potangle)
    self.pubbs.publish(self.stretch)
    self.pubpres.publish(self.pressure)

  def ftscb(self, message):
    self.mz = message.data[-1] 
 

class MotorController:
  def __init__(self, control_target, data_manager):
    self.motor = MX64Control(b"/dev/ttyS0", 57600, [control_target])
    self.motor_s = [0, 0, 1] # [extract, inject, stop]
    self.target = control_target

    self.dm = data_manager # should be the instance of DataManager
        
  def get_position(self):
    return self.motor.getPosition(self.target)

  def rotate_servo(self, value):
    if value == 0: # stop
      self.motor.setTorque(1024, self.target)
      self.motor_s = [0, 0, 1]
    elif value > 0: # injection
      self.motor.setTorque(1024 + value, self.target)
      self.motor_s = [0, 1, 0]
    elif value < 0: # extraction
      self.motor.setTorque(-value, self.target) 
      self.motor_s = [1, 0, 0]
   
  def step_diff(self, goal, diff_type="servo", max_pressure=30, speed=1):
    cp = 0 # current position
    diff = 0 # difference
    allowable_err = 0 

    if diff_type == "servo":
      cp = self.motor.getPosition(self.target)
      diff = goal - cp
      allowable_err = 10

    elif diff_type == "pot": # potentiometer
      diff = self.dm.potangle - goal 
      allowable_err = 1

    elif diff_type == "pressure":
      diff = self.dm.pressure - goal
      allowable_err = 0.2

    print diff
    if abs(diff) <= allowable_err:
      self.rotate_servo(0)
      isdone = True
    elif diff > 0:
      print "extract"
      if self.dm.pressure < -25:
        print "pressure becomes under -" + str(max_pressure) + "kPa"
        self.rotate_servo(0)
        isdone = True
      else:  
        self.rotate_servo(-3*speed)
        isdone = False
    elif diff < 0:
      print "inject"
      if self.dm.pressure > max_pressure:
        print "presssure becomes over" + str(max_pressure)+ "kPa"
        self.rotate_servo(0)
        isdone = True
      else:  
        self.rotate_servo(4*speed)
        isdone = False
    return isdone

  def servo_pos_control(self, goal, diff_type="servo", speed=1):
    while not self.step_diff(goal, diff_type=diff_type, speed=speed):
      time.sleep(0.01)


