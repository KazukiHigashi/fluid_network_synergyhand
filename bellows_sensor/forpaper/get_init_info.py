#!/usr/bin/env python
## coding: UTF-8

import rospy
import time
import sys
import os
import datetime
import csv
import yaml
import click
from serial_dx_test.dx_comm import MX64Control
from bellows_experiment_utilities.bellows_utils import Predictor, DataManager, MotorController

import numpy as np

from stage_py2 import StageControl

label2id = {"A": 0, "B": 1, "C": 2, "D": 3, "F": 4, "G": 5}

@click.command()
@click.argument("config_path")
def main(config_path):
  rospy.init_node("evaluate", disable_signals=True)

  with open(os.path.join("/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/forpaper/yaml/", config_path)) as yml:
    config = yaml.load(yml)


  axis = "THETA_STAGE"

  joint_labels = list(config["weights"].keys())
 
  jid = config["joint_id"]

  max_stretch = {}
  min_stretch = {}
  for j_label in joint_labels:
    max_stretch[j_label] = 1e10
    min_stretch[j_label] = -10
 
  dm = DataManager(max_stretch=max_stretch, min_stretch=min_stretch, joint_labels=joint_labels)

  mc = MotorController(control_target=1, data_manager=dm)
  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)
  t_stage.absolute_move(axis="THETA_STAGE", mm=0, speed=30)
 
  print "\n\n"
  time.sleep(1)
  # Check the configure for 10 seconds!
  print "Current pot: {}".format(dm.potangle[jid])
  print "Current pressure: {} [kPa]".format(dm.pressure)
  print "Current stretch: {}".format(dm.stretch[jid])
  time.sleep(10)
   
  max_stretch = -10
  min_stretch = 1e10
  inv = False
  max_pot = -10 
  min_pot = 1e10

  try:
    t_stage.absolute_move(axis="THETA_STAGE", mm=0, speed=30)
    inv_check_min = 0
    for goal in [-15, 30, -20, 35]:
      while not mc.step_diff(goal=goal, jid=jid, diff_type="pressure", max_pressure=40, speed=10):
        if dm.potangle[jid] < min_pot:
          min_pot = dm.potangle[jid]
        if dm.potangle[jid] > max_pot:
          max_pot = dm.potangle[jid]
        if dm.stretch[jid] < min_stretch:
          min_stretch = dm.stretch[jid]
        if dm.stretch[jid] > max_stretch:
          max_stretch = dm.stretch[jid]
        time.sleep(0.03)
      
      time.sleep(0.5)
      if goal == -15:
        inv_check_min = dm.potangle[jid]
      elif goal == 30:
        if dm.potangle[jid] > inv_check_min:
          inv = False
        else:
          inv = True 
      time.sleep(0.5)

    mc.servo_pos_control(goal=0, jid=jid, diff_type="pressure", speed=3)
    time.sleep(5)
    t_stage.absolute_move(axis="THETA_STAGE", mm=0, speed=30)
    print("**RESULT**")
    print("max stretch:{}, min stretch:{}, inv:{}, min potentio:{}".format(max_stretch, min_stretch, inv, max_pot if inv else min_pot))
 
 
  except KeyboardInterrupt:
    print "interrupted"
    t_stage.absolute_move(axis="THETA_STAGE", mm=0, speed=30)
    mc.servo_pos_control(goal=0, jid=jid, diff_type="pressure", speed=3)
    rospy.signal_shutdown("finish")
 
if __name__ == "__main__":
  main()    
