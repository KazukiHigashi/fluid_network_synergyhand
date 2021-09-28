#!/usr/bin/env python
## coding: UTF-8

import rospy
import time
import signal
import sys
import os
import datetime
import csv
import numpy as np
from serial_dx_test.dx_comm import MX64Control
from std_msgs.msg import Float32MultiArray, Int16, Float32

from stage_py2 import StageControl

target = 1
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

def is_outlier(data_list, data):
  data_list = np.abs(np.array(data_list) - data)
  if np.min(data_list) > 150:
    return True # OUTLIER
  else:
    return False # NOT OUTLIER 

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
    allowable_err = 2

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

def loop(pot_min, servo_pulses, filebasepath="~/", mmlist=[0, 10], ntrials=5):
  global potangle, mz, servo_s
  # min angle : 6600 pulses of the servo, 45mL in the syringe
  # max angle : -3151 pulses, 
  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract
  for mm in mmlist:
    if mm < 0 or mm > 80:
      print "illegal angles"
      break

    os.makedirs(os.path.join(filebasepath, str(mm)))
    t_stage.absolute_move(axis=axis, mm=mm, speed=50)
    time.sleep(3)
    for cnt in range(ntrials):
      filename = os.path.join(filebasepath, str(mm), str(cnt)+".csv")
      servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara
      time.sleep(2)
      with open(filename, "w") as f:
        writer = csv.writer(f)
        writer.writerow(["Time", "Syringe Servo Pulses", "Potentiometer", "Moment", "Extraction", "Injection", "Stop"])
        
        # record initial values
        init_mz = mz
        init_pot = potangle
        init_time = time.time()

        outlier = True
        srv_list = []
        while outlier:
          srv_list = []
          for i in range(5):
            print(srv_list)
            srv_list = np.append(srv_list, motor.getPosition(target))
            time.sleep(0.05)
          outlier = False
          for i in range(5):
            if is_outlier(np.delete(srv_list, i), srv_list[i]):
              print("hello")
              outlier = True
        init_srv = np.average(srv_list)

        goals = [(init_srv-servo_pulses, "servo"), (pot_min, "pot")] # initial and fully rotated position of potentiometer

        for goal in goals:
          print(goal)
          i = 0
          while not step_diff(goal[0], diff_type=goal[1]):
            t_srv = motor.getPosition(target)
            
            while is_outlier(srv_list, t_srv):
              t_srv = motor.getPosition(target)

            srv_list = np.delete(np.append(srv_list,t_srv), 0)

            writer.writerow([time.time()-init_time, t_srv, potangle, mz] + servo_s)
            print servo_s
            print "{}th time : {}".format(i, time.time()-init_time)
            print "{}th pot angle : {}".format(i, potangle-init_pot)
            print "{}th servo angle : {}".format(i, t_srv-init_srv)
            print "{}th mz : {:.5f} [Nm]".format(i, mz-init_mz)
            i = i + 1
            
            if i >= 1000:
              print "ERROR: pot value cannot converge into the goal"
              rospy.signal_shutdown("finish")
              motor.setTorque(1024, target)
              sys.exit(1)
            if abs(mz) >= 0.20:
              print "ERROR: z-moment exceeded the safe value"
              rospy.signal_shutdown("finish")
              motor.setTorque(1024, target)
              sys.exit(1)
         
            time.sleep(0.05)
          
          if goal[0] == goals[0][0]:
            # keep the max torque state
            rotate_servo(0)
            for i in range(100):
              t_srv = motor.getPosition(target)
              while is_outlier(srv_list, t_srv):
                t_srv = motor.getPosition(target)
              srv_list = np.delete(np.append(srv_list,t_srv), 0)
              writer.writerow([time.time()-init_time, t_srv, potangle, mz] + servo_s)
              print servo_s
              print "{}th pot angle : {}".format(i, potangle-init_pot)
              print "{}th servo angle : {}".format(i, t_srv-init_srv)
              print "{}th mz : {:.5f} [Nm]".format(i, mz-init_mz)
              time.sleep(0.05)
        time.sleep(2)      

if __name__ == "__main__":
  rospy.init_node("evaluate", disable_signals=True)
  motor = MX64Control(b"/dev/ttyS0", 57600, [target])
  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)


  today = datetime.datetime.today().strftime("%y%b%d%I%M")
  
  if len(sys.argv) > 1:
    today = today + sys.argv[1]

  filebasepath = os.path.join("/home/ubuntu/workspace/src/fluid_network_synergyhand/eval_jabara/csv", today) 

  print "csv gonna be saved at " + filebasepath

  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract

  mmlist = range(0, 63, 3)

  # A : 255
  # B : 267
  # C : 260
  
  try:
    loop(pot_min=260, servo_pulses=8000, filebasepath=filebasepath, mmlist=mmlist , ntrials=3)
  except:
    rospy.signal_shutdown("finish")
    motor.setTorque(1024, target)
    sys.exit(1)

