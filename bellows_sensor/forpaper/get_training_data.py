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


def step_method(dm, mc, predictor, init_pot, init_time, init_mz, writer):
  input_data = np.array([[dm.potangle-init_pot, dm.normed_stretch, dm.pressure] + mc.motor_s])
  pred = predictor.forward_propagation(input_data)[0]
  writer.writerow([time.time()-init_time, mc.get_position(), dm.potangle, dm.stretch, dm.pressure, dm.mz, pred] + mc.motor_s)
  dm.pubpred.publish(-1000*pred)
  dm.pubmz.publish(-1000*(dm.mz-init_mz))

def loop(pot_min, filepath, dm, mc, predictor, t_stage, mmlist=[0, 10], max_pressure_list=[10, 20, 30]):
  mc.servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara
  os.makedirs(filepath)

  # record all data
  for mm in mmlist:
    os.makedirs(os.path.join(filepath, str(mm)))
    t_stage.absolute_move(axis="THETA_STAGE", mm=mm, speed=30)
 
    time.sleep(7)
    print(max_pressure_list)
    for n, max_pressure in enumerate(max_pressure_list):
      # To make initial condition stable
      print("pressure -> 20")
      mc.servo_pos_control(goal=5, diff_type="pressure", speed=3)
      time.sleep(1)
      print("pressure -> -15")
      mc.servo_pos_control(goal=-5, diff_type="pressure", speed=3)
      
      time.sleep(5)
      with open(os.path.join(filepath, str(mm), str(n)+".csv"), "w") as f: 
        writer = csv.writer(f)
        writer.writerow(["Time", "Syringe Servo Pulses", "Potentiometer", "Stretch", "Pressure", "Moment", "Prediction", "Extraction", "Injection", "Stop"])

        init_srv = mc.get_position()
        init_time = time.time()
        init_pot = dm.potangle
        init_mz = dm.mz

        maxpulse = 30000 - mm*150
        for goal, diff_type in [(init_srv-maxpulse, "servo"), (-15, "pressure")]:
          for i in range(300):
            step_method(dm, mc, predictor, init_pot, init_time, init_mz, writer)
            time.sleep(0.03)
 
          while not mc.step_diff(goal=goal, diff_type=diff_type, max_pressure=max_pressure):
            step_method(dm, mc, predictor, init_pot, init_time, init_mz, writer)
            time.sleep(0.03)
 
          for i in range(300):
            step_method(dm, mc, predictor, init_pot, init_time, init_mz, writer)
            time.sleep(0.03)

@click.command()
@click.argument("config_path")
@click.option("--sign", "-s", default="")
def main(config_path, sign):
  rospy.init_node("evaluate", disable_signals=True)
 
  MAXSTRETCH = 856.0
  MINSTRETCH = 150.0

  with open(os.path.join("/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/forpaper/yaml/", config_path)) as yml:
    config = yaml.load(yml)

  axis = "THETA_STAGE"

  print config

  dm = DataManager(max_stretch=MAXSTRETCH, min_stretch=MINSTRETCH)
  weights_path = config["weights"]["path"]
  predictor = Predictor(weights_path=weights_path)
  mc = MotorController(control_target=1, data_manager=dm)

  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)

  ym = datetime.datetime.today().strftime("%y%b")
  today = datetime.datetime.today().strftime("%d_%H%M%S")

  today = today + "_" + sign

  filepath = os.path.join("/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/csv", ym, today)
  
  if config["repeatability"]["stage"] is not None:
    mmlist = [config["repeatability"]["stage"]]
    max_pressure_list = [config["repeatability"]["pressure"]]*config["repeatability"]["num"]
  else:  
    mmlist = range(config["stage"]["max"], config["stage"]["min"], -config["stage"]["width"])
    max_pressure_list = range(config["pressure"]["min"], config["pressure"]["max"], config["pressure"]["width"])
  
  print "\n\n"
  time.sleep(1)

  # Check the configure for 10 seconds!
  print "mmlist: " + str(mmlist)
  print "pressure list: " + str(max_pressure_list)
  
  print "Current pot: {}".format(dm.potangle)
  print "potmin: {}".format(config["potmin"])
  print "Current pressure: {} [kPa]".format(dm.pressure)
  print "Current stretch: {}".format(dm.stretch)
  time.sleep(10)
   

  try:
    loop(pot_min=config["potmin"], filepath=filepath, mmlist=mmlist, dm=dm, mc=mc, predictor=predictor, t_stage=t_stage, max_pressure_list=max_pressure_list)
  except KeyboardInterrupt:
    print "interrupted"
    rospy.signal_shutdown("finish")

if __name__ == "__main__":
  main()    
