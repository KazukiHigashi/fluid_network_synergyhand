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

def step_method(dm, mc, predictor, init_pot, init_time, init_mz, writer, joint_labels, jid):
  # print(input_data)
  for joint_l in joint_labels:
    input_data = np.array([[dm.potangle[label2id[joint_l]], dm.normed_stretch[joint_l], dm.pressure] + mc.motor_s])
    raw_pred = predictor[joint_l].forward_propagation(input_data)[0]
    dm.add_pred(raw_pred, joint_l)
    prediction = dm.get_ma_pred(joint_l)
    dm.pubpred[joint_l].publish(1000*prediction)
  writer.writerow([time.time()-init_time, mc.get_position(), dm.potangle[jid], dm.stretch[jid], dm.pressure, dm.mz, prediction] + mc.motor_s)
  dm.pubmz.publish(-1000*(dm.mz-init_mz))

def loop(pot_min, filepath, dm, mc, predictor, t_stage, mmlist=[0, 10], max_pressure_list=[10, 20, 30], pot_inv=False, jid=0, joint_labels=["D"], init_stage_angle=0):
  # jid
  #   0:A
  #   1:B
  #   2:C
  #   3:D

  mc.servo_pos_control(goal=pot_min, jid=jid, diff_type="pot", pot_inv=pot_inv) # restore the position of the jabara
  os.makedirs(filepath)

  # record all data
  for i, mm in enumerate(mmlist):
    os.makedirs(os.path.join(filepath, str(mm)))
    t_stage.absolute_move(axis="THETA_STAGE", mm=init_stage_angle, speed=30)
    time.sleep(6)
    print("pressure -> 30")
    mc.servo_pos_control(goal=30, jid=jid, diff_type="pressure", speed=20)
    time.sleep(0.5)
    print("pressure -> -15")
    mc.servo_pos_control(goal=-15, jid=jid, diff_type="pressure", speed=20)

    t_stage.absolute_move(axis="THETA_STAGE", mm=mm, speed=30)
    time.sleep(6)
    print(max_pressure_list)
    for n, max_pressure in enumerate(max_pressure_list[i]):
      dm.clear_ma()
      # To make initial condition stable
      print("pressure -> 15")
      mc.servo_pos_control(goal=15, jid=jid, diff_type="pressure", speed=20)
      time.sleep(0.5)
      print("pressure -> -15")
      mc.servo_pos_control(goal=-15, jid=jid, diff_type="pressure", speed=20)
      
      time.sleep(3)
      with open(os.path.join(filepath, str(mm), str(n)+".csv"), "w") as f: 
        writer = csv.writer(f)
        writer.writerow(["Time", "Syringe Servo Pulses", "Potentiometer", "Stretch", "Pressure", "Moment", "Prediction", "Extraction", "Injection", "Stop"])

        init_srv = mc.get_position()
        init_time = time.time()
        init_pot = dm.potangle[jid]
        init_mz = dm.mz

        maxpulse = 30000 - mm*150
        for goal, diff_type in [(-0.1, "predict"), (-15, "pressure")]:
          for i in range(300):
            step_method(dm, mc, predictor, init_pot, init_time, init_mz, writer, joint_labels, jid)
            time.sleep(0.03)
 
          while not mc.step_diff(goal=goal, jid=jid, diff_type=diff_type, max_pressure=max_pressure, speed=20, joint_l="A"):
            step_method(dm, mc, predictor, init_pot, init_time, init_mz, writer, joint_labels, jid)
            time.sleep(0.03)
 
          for i in range(300):
            step_method(dm, mc, predictor, init_pot, init_time, init_mz, writer, joint_labels, jid)
            time.sleep(0.03)

@click.command()
@click.argument("config_path")
@click.option("--sign", "-s", default="")
def main(config_path, sign):
  rospy.init_node("evaluate", disable_signals=True)

  with open(os.path.join("/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/forpaper/yaml/", config_path)) as yml:
    config = yaml.load(yml)

  joint_labels = list(config["weights"].keys())

  axis = "THETA_STAGE"

  print config

  predictor = {}
  max_stretch = {}
  min_stretch = {}
  for j_label in joint_labels:
    predictor[j_label] = Predictor(weights_path=config["weights"][j_label])
    max_stretch[j_label] = config["stretch"][j_label]["max"]
    min_stretch[j_label] = config["stretch"][j_label]["min"]
    
  com_pres_range  = range(config["pressure"]["min"], config["pressure"]["max"], config["pressure"]["width"])
  mmlist = range(config["stage"]["min"], config["stage"]["max"], config["stage"]["width"])
  if config["init"]["num"] != "None":
    max_pressure_list = [[config["init"]["pressure"]]*config["init"]["num"] + com_pres_range]
    max_pressure_list = max_pressure_list + [com_pres_range]*(len(mmlist)-1)
  else:
    max_pressure_list = [com_pres_range]*len(mmlist)
  
  # Configure test data
  if config["test_stage"]["min"] != "None":
    test_mmlist = range(config["test_stage"]["min"], config["test_stage"]["max"], config["test_stage"]["width"])
    mmlist = mmlist + test_mmlist
    test_max_pressure_list = [range(config["test_pressure"]["min"], config["test_pressure"]["max"], config["test_pressure"]["width"])]*(len(test_mmlist))
    max_pressure_list = max_pressure_list + test_max_pressure_list 

  pot_inv = config["inv"]
  jlabel = config["joint_label"]
  jid = label2id[jlabel]
  init_stage_angle = config["stage"]["min"]

  # Start all services
  dm = DataManager(max_stretch=max_stretch, min_stretch=min_stretch, joint_labels=joint_labels)
  mc = MotorController(control_target=1, data_manager=dm)
  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)

  print "\n\n"
  time.sleep(1)
  # Check the configuration for 10 seconds!
  print "mmlist: " + str(mmlist)
  print "pressure list: " + str(max_pressure_list)
  print "Current pot: {}".format(dm.potangle[jid])
  print "potmin: {}".format(config["potmin"])
  print "Current pressure: {} [kPa]".format(dm.pressure)
  print "Current stretch: {}".format(dm.stretch[jid])
  time.sleep(10)
   
  # file name 
  ym = datetime.datetime.today().strftime("%y%b")
  today = datetime.datetime.today().strftime("%d_%H%M%S")
  today = today + "_" + sign
  filepath = os.path.join("/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/csv", ym, today)
 
  try:
    loop(pot_min=config["potmin"], filepath=filepath, mmlist=mmlist, dm=dm, mc=mc, predictor=predictor, t_stage=t_stage, max_pressure_list=max_pressure_list, pot_inv=pot_inv, jid=jid, joint_labels=joint_labels, init_stage_angle=init_stage_angle)
    t_stage.absolute_move(axis="THETA_STAGE", mm=init_stage_angle, speed=30)
    mc.servo_pos_control(goal=0, jid=jid, diff_type="pressure", speed=30)
 
  except KeyboardInterrupt:
    print "interrupted"
    t_stage.absolute_move(axis="THETA_STAGE", mm=init_stage_angle, speed=30)
    mc.servo_pos_control(goal=0, jid=jid, diff_type="pressure", speed=30)
    rospy.signal_shutdown("finish")
 
if __name__ == "__main__":
  main()    
