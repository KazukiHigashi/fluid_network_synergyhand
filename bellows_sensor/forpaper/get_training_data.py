#!/usr/bin/env python
## coding: UTF-8

import rospy
import time
import sys
import os
import datetime
import csv
from serial_dx_test.dx_comm import MX64Control
from bellows_experiment_utilities.bellows_utils import Predictor, DataManager, MotorController
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Float32, Int16

import numpy as np

from stage_py2 import StageControl


def step_method(dm, mc, predictor):
  input_data = np.array([[dm.potangle-init_pot, dm.normed_stretch, dm.pressure] + mc.motor_s])
  pred = predictor.forward_propagation(input_data)[0]
  writer.writerow([time.time()-init_time, mc.get_position(), dm.potangle, dm.stretch, dm.pressure, dm.mz, pred] + mc.motor_s)
  dm.pubpred.publish(-1000*pred)
  dm.pubmz.publish(-1000*(dm.mz-init_mz))

def loop(pot_min, filepath, dm, mc, mlist=[0, 10], max_pressure_list=[10, 20, 30]):
  mc.servo_pos_control(goal=pot_min, diff_type="pot") # restore the position of the jabara
  os.makedirs(filepath)

  # record all data
  for mm in mmlist:
    os.makedirs(os.path.join(filepath, str(mm)))
    t_stage.absolute_move(axis=axis, mm=mm, speed=30)
 
    time.sleep(7)
    print(max_pressure_list)
    for n, max_pressure in enumerate(max_pressure_list):
      # To make initial condition stable
      print("pressure -> 20")
      mc.servo_pos_control(goal=15, diff_type="pressure")
      time.sleep(1)
      print("pressure -> -15")
      mc.servo_pos_control(goal=-15, diff_type="pressure")
      
      time.sleep(5)
      with open(os.path.join(filepath, str(mm), str(n)+".csv"), "w") as f: 
        writer = csv.writer(f)
        writer.writerow(["Time", "Syringe Servo Pulses", "Potentiometer", "Stretch", "Pressure", "Moment", "Prediction", "Extraction", "Injection", "Stop"])

        init_srv = mc.get_position()
        init_time = time.time()
        init_pot = dm.potangle
        init_mz = dm.mz

        maxpulse = 11000 - mm*75
        maxpulse = 17000 - mm*150
        for goal, diff_type in [(init_srv-maxpulse, "servo"), (-15, "pressure")]:
          for i in range(300):
            step_method(dm, mc, predictor)
            time.sleep(0.03)
 
          while not mc.step_diff(goal=goal, diff_type=diff_type, max_pressure=max_pressure):
            step_method(dm, mc, predictor)
            time.sleep(0.03)
 
          for i in range(300):
            step_method(dm, mc, predictor)
            time.sleep(0.03)

if __name__ == "__main__":
  rospy.init_node("evaluate", disable_signals=True)
 
  MAXSTRETCH = 856.0
  MINSTRETCH = 150.0

  axis = "THETA_STAGE"

  dm = DataManager(max_stretch=MAX_STRETCH, min_stretch=MIN_STRETCH)
  weights_path = '/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/scripts/weights0113_py2'
  predictor = Predictor(weights_path=weights_path)
  mc = MotorController(control_target=1, data_manager=dm)

  t_stage = StageControl(comport=b"/dev/ttyUSB0", baudrate=9600)

  ym = datetime.datetime.today().strftime("%y%b")
  today = datetime.datetime.today().strftime("%d%I%M")

  if len(sys.argv) > 1:
    today = today + sys.argv[1]

  filepath = os.path.join("/home/ubuntu/workspace/src/fluid_network_synergyhand/bellows_sensor/csv", ym, today)
  
  # mmlist = range(60, 0, -7)
  mmlist = range(75, -5, -5)
  print(mmlist)
  max_pressure_list = [5, 10, 15, 20, 25]

  try:
    loop(pot_min=280, filepath=filepath, mmlist=mmlist, dm=dm, mc=mc, max_pressure_list=max_pressure_list)
  except KeyboardInterrupt:
    print "interrupted"
    rospy.signal_shutdown("finish")
