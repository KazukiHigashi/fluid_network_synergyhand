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

label2id = {"a": 0, "b": 1, "c": 2, "d": 3, "f": 4, "g": 5}

class Predictor:
  def __init__(self, weights_path):
    self.w = None
    with open(weights_path, "rb") as f:
      self.w = pickle.load(f)
    
  def forward_propagation(self, x):
    def norm(x):
        return np.nan_to_num((x - self.w[0]) / np.sqrt(self.w[1]))
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

class DualBellowsModel:
  # TODO: supports for multiple joints
  def __init__(self, weights_path, deg90_potentio, deg0_potentio, kb, delta):
    self.deg90_pot = deg90_potentio
    self.deg0_pot = deg0_potentio

    invmatrix = np.linalg.inv(np.array([[self.deg90_pot, 1], [self.deg0_pot, 1]]))
    angleab = np.dot(invmatrix, np.array([[np.pi/2], [np.pi/180]]))
    print(angleab)
    self.anglea = angleab[0][0]
    self.angleb = angleab[1][0]

    self.km_predictor = Predictor(weights_path=weights_path)

    self.A = np.pi*(0.010/2.0)**2

    self.r = 0.0188

    self.Kb = kb
    self.delta = delta

    self.subcontact = rospy.Subscriber("contact", Float32MultiArray, self.contactcb)

    self.mode = "bc"
    self.st_contacted = None
    self.pot_contacted = None
    self.pressure_grasped = None
    self.stretch_highest = None

  def to_angle(self, x):
    return self.anglea*x + self.angleb

  def to_force(self, x):
    return 1000*x*self.A

  # Stretch sensor's length when contacting to the object
  def lm0(self, sens_st):
    if self.st_contacted is None:
      return sens_st
    else:
      return self.st_contacted

  def contactcb(self, message):
    # When the joint detects contact between the link and an obstacle, 
    # this method will be called from the DataManager.contact_check()
    # message : [joint_l, self.stretch, self.potangle[label2id[joint_l]]]
    # joint_l = int(message[0])
    print("@CONTACTED!@")
    self.mode = "ac"
    self.st_contacted = message.data[0]
    self.pot_contacted = message.data[1]

  def grasped(self, pressure_grasped, stretch_highest):
    # This method should be called from the main code when completing a grasp.
    print("@GRASPED!@")
    self.mode = "ag"
    if self.pressure_grasped is None:
      self.pressure_grasped = pressure_grasped
      self.stretch_highest = stretch_highest

  def init(self):
    self.mode = "bc"
    self.st_contacted = None
    self.pot_contacted = None
    self.pressure_grasped = None
    self.stretch_highest = None


  def predict(self, pressure, potentio, stretch):
    # mode
    #  - bc : before contacted
    #  - ac : after contacted
    #  - ag : after grasped
    if pressure < 0:
      return 0

    if self.mode == "bc":
      torque = self.to_force(pressure) - self.Kb*(self.to_angle(potentio) - self.to_angle(self.deg0_pot))*self.r - self.delta
    elif self.mode == "ac":
      print()
      print(self.to_angle(self.pot_contacted), self.pressure_grasped)
      print(self.to_force(pressure), self.to_angle(potentio), stretch)
      km = self.km_predictor.forward_propagation(np.array([[self.to_angle(self.pot_contacted), pressure]]))[0]
      torque = self.to_force(pressure) - self.Kb*(self.to_angle(potentio) - self.to_angle(self.deg0_pot))*self.r - self.delta - km*(stretch - self.lm0(stretch))
      print "#{},{}".format(km, torque*self.r)
    elif self.mode == "ag":
      print()
      print(pressure, potentio, stretch)
      print(self.to_angle(self.pot_contacted), self.pressure_grasped)
      print(self.to_force(pressure), self.to_angle(potentio), stretch, self.stretch_highest)
      km = self.km_predictor.forward_propagation(np.array([[self.to_angle(self.pot_contacted), self.pressure_grasped]]))[0]
      torque = self.to_force(pressure) - self.Kb*(self.to_angle(potentio) - self.to_angle(self.deg0_pot))*self.r - self.delta - km*(self.stretch_highest - self.lm0(stretch))
      print "#{},{}".format(km, torque*self.r)
    return torque*self.r


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
  def __init__(self, max_stretch, min_stretch, joint_labels):
    self.subpot = rospy.Subscriber("publish_pot_angle", Int16MultiArray, self.potcb)
    self.subfts = rospy.Subscriber("ftsensor_raw", Float32MultiArray, self.ftscb)

    self.potangle = ()
    self.mz = 0
    self.stretch = []
    self.pulses = 0
    self.pressure = 0
    
    self.joint_labels = joint_labels
    self.ps = PressureSensor(-30, 50, -100, 100)

    self.ma_pres = []
    self.ma_strc = []

    # self.ma_pred = []

    self.max_stretch = max_stretch
    self.min_stretch = min_stretch

    self.pubpot = rospy.Publisher("potentiometer", Int16MultiArray, queue_size=10)
    self.pubbs = rospy.Publisher("bellows_sensor", Int16MultiArray, queue_size=10)
    self.pubpres = rospy.Publisher("pressure", Float32, queue_size=10)
    self.pubmz = rospy.Publisher("mz", Float32, queue_size=10)
    self.pubcontact = rospy.Publisher("contact", Float32MultiArray, queue_size=10)
    
    self.pubpred_dnn = {}
    self.ma_pred_dnn = {}
    self.pubpred_dbm = {}
    self.ma_pred_dbm = {}
    self.pot_list = {}
    self.diff_pot = {}
    self.normed_stretch = {}
    self.is_moved = {}

    for joint_l in self.joint_labels:
      self.pubpred_dnn[joint_l] = rospy.Publisher("prediction_DNN_"+joint_l, Float32, queue_size=10)
      self.pubpred_dbm[joint_l] = rospy.Publisher("prediction_DBM_"+joint_l, Float32, queue_size=10)
      self.ma_pred_dnn[joint_l] = []
      self.ma_pred_dbm[joint_l] = []
      self.normed_stretch[joint_l] = 0
      self.pot_list[joint_l] = []
      self.diff_pot[joint_l] = []
      self.is_moved[joint_l] = False

  def potcb(self, message):
    self.potangle = message.data[1:7]
    self.stretch = message.data[7:13]
    self.pressure = self.ps.digital2pressure(message.data[0])

    self.pubpot.publish(Int16MultiArray(data=message.data[1:7]))
    
    # moving average
    self.ma_pres.append(self.pressure)
    self.ma_strc.append(self.stretch)

    self.pressure = np.mean(self.ma_pres[-5:], axis=0)
    self.stretch = np.mean(self.ma_strc[-5:], axis=0)

    for joint_l in self.joint_labels:
      self.pot_list[joint_l].append(self.potangle[label2id[joint_l]])
      self.normed_stretch[joint_l] = float(self.stretch[label2id[joint_l]]-self.min_stretch[joint_l])/(self.max_stretch[joint_l]-self.min_stretch[joint_l])
      self.set_diff_pot(joint_l)
      self.contact_check(joint_l)

    self.pubbs.publish(Int16MultiArray(data=self.stretch))
    self.pubpres.publish(self.pressure)

  def add_pred(self, pred, joint_l, predtype="dnn"):
    if predtype == "dnn":
      for joint_l in self.joint_labels:
        self.ma_pred_dnn[joint_l].append(pred)
    elif predtype == "dbm":
      for joint_l in self.joint_labels:
        self.ma_pred_dbm[joint_l].append(pred)

  def get_ma_pred(self, joint_l, predtype="dnn"):
    if predtype == "dnn":
      return np.mean(self.ma_pred_dnn[joint_l][-5:], axis=0)
    elif predtype == "dbm":
      return np.mean(self.ma_pred_dbm[joint_l][-5:], axis=0)
  
  def set_diff_pot(self, joint_l):
    if len(self.pot_list[joint_l]) >= 10:
      self.diff_pot[joint_l].append(self.potangle[label2id[joint_l]] - self.pot_list[joint_l][-10])

  def contact_check(self, joint_l):
    if np.sum(np.abs(self.diff_pot[joint_l][-5:])) <= 5 and self.is_moved[joint_l]:
      print(joint_l, self.stretch[label2id[joint_l]], self.potangle[label2id[joint_l]])
      self.pubcontact.publish(data=np.array([self.normed_stretch[joint_l], self.potangle[label2id[joint_l]]]))
      self.is_moved[joint_l] = False
      return True
    elif np.sum(np.abs(self.diff_pot[joint_l][-5:])) > 10 and not self.is_moved[joint_l]:
      self.is_moved[joint_l] = True
      return False
    else:
      return False

  def clear_ma(self):
    self.ma_pres = []
    self.ma_strc = []
    for joint_l in self.joint_labels:
      self.ma_pred_dnn[joint_l] = []
      self.ma_pred_dbm[joint_l] = []
      self.diff_pot[joint_l] = []
      self.pot_list[joint_l] = []
      self.is_moved[joint_l] = False

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
   
  def step_diff(self, goal, jid, diff_type="servo", max_pressure=30, speed=1, inv=False, joint_l="A"):
    cp = 0 # current position
    diff = 0 # difference
    allowable_err = 0
    sign = 1 

    if diff_type == "servo":
      cp = self.motor.getPosition(self.target)
      diff = goal - cp
      allowable_err = 10

    elif diff_type == "pot": # potentiometer
      diff = self.dm.potangle[jid] - goal
      allowable_err = 1
      if inv:
        sign = -1

    elif diff_type == "pressure":
      diff = self.dm.pressure - goal
      allowable_err = 0.2
    
    elif diff_type == "prediction":
      print "prediction {}, goal {}".format(self.dm.get_ma_pred(joint_l, "dbm"), goal)
      diff = -(-self.dm.get_ma_pred(joint_l, "dbm") - goal)
      allowable_err = 0.003
    else:
      print "!!!caution diff type is illegal!!!"  

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
        self.rotate_servo(sign*-3*speed)
        isdone = False
    elif diff < 0:
      print "inject"
      if self.dm.pressure > max_pressure:
        print "presssure becomes over" + str(max_pressure)+ "kPa"
        self.rotate_servo(0)
        isdone = True
      else:  
        self.rotate_servo(sign*4*speed)
        isdone = False
    return isdone

  def servo_pos_control(self, goal, jid, diff_type="servo", speed=1, pot_inv=False):
    while not self.step_diff(goal, jid, diff_type=diff_type, speed=speed, inv=pot_inv):
      time.sleep(0.01)


