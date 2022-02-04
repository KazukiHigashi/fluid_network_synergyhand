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

cont = True

def analog2p(analog):
  # return 1000*(0.011*(analog*0.02444988-5.01222494)-0.1) # [kPa]
  return 1000*(0.011*(analog*0.00611124694+6.7469437653)-0.1) # [kPa]


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


def loop(pot_min, pot_max, filepath, ntrials=1):
  global potangle, stretch, mz, pressure, servo_s
  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract
  servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara

  os.makedirs(filepath)
  
  # # record data when the pressure is 0
  # servo_pos_control(goal=0, diff_type="pressure")
  # time.sleep(3)
  # with open(os.path.join(filepath, "zero_pressure.csv"), "w") as f:
  #   writer = csv.writer(f)
  #   writer.writerow(["Time", "Syringe Servo Pulses", "Potentiometer", "Stretch", "Pressure", "Moment", "Extraction", "Injection", "Stop"])
  #   t_srv = motor.getPosition(target)
  #   writer.writerow([time.time(), t_srv, potangle, stretch, pressure, mz] + servo_s)

  # record all data
  for cnt in range(ntrials):
    servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara
    with open(os.path.join(filepath, str(cnt)+".csv"), "w") as f: 
      writer = csv.writer(f)
      writer.writerow(["Time", "Syringe Servo Pulses", "Potentiometer", "Stretch", "Pressure", "Moment", "Extraction", "Injection", "Stop"])

      init_srv = motor.getPosition(target)
      init_time = time.time()

      maxpulse = 11000 - 50
      for goal in [pot_min, pot_max]:
        while not step_diff(goal, diff_type="pot"):
          t_srv = motor.getPosition(target)
          writer.writerow([time.time()-init_time, t_srv, potangle, stretch, pressure, mz] + servo_s)
          time.sleep(0.03)
        
        for i in range(200):
          t_srv = motor.getPosition(target)
          writer.writerow([time.time()-init_time, t_srv, potangle, stretch, pressure, mz] + servo_s)
          time.sleep(0.03)
   
pubpot = rospy.Publisher("potentiometer", Int16, queue_size=10)
pubbs = rospy.Publisher("bellows_sensor", Int16, queue_size=10)
pubpres = rospy.Publisher("pressure", Int16, queue_size=10)
pubmz = rospy.Publisher("mz", Float32, queue_size=10)

if __name__ == "__main__":
  rospy.init_node("evaluate", disable_signals=True)
  motor = MX64Control(b"/dev/ttyS0", 57600, [target])
  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)

  # motor.setTorque(1027, 2) # inject
  # motor.setTorque(2, 2) # extract
  
  ym = datetime.datetime.today().strftime("%y%b")
  today = datetime.datetime.today().strftime("%d%I%M")

  if len(sys.argv) > 1:
    today = today + sys.argv[1]

  filepath = os.path.join("/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/csv", ym, today)
  
  t_stage.absolute_move(axis=axis, mm=0, speed=30)

  try:
    loop(pot_min=280, pot_max=500, filepath=filepath, ntrials=50)
  except KeyboardInterrupt:
    print "interrupted"
