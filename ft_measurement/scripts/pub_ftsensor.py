#!/usr/bin/env python
## coding: UTF-8

from ftsensor_comm import get_model_num, get_latest_data
import spidev
import time
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray

rospy.init_node("ftsensor_publisher")

ft_list = []

meanrange = 3

pubraw = rospy.Publisher("ftsensor_raw", Float32MultiArray, queue_size=10)
pubmean = rospy.Publisher("ftsensor_mean", Float32MultiArray, queue_size=10)

while True:
  c_ft = get_latest_data()

  # if illegal packet comes
  if c_ft is None:
    continue

  if len(ft_list) >= meanrange:
    ft_list.pop(0)

  ft_list.append(c_ft)

  raw_data = Float32MultiArray()
  mean_data = Float32MultiArray()


  raw_data.data = c_ft
  if len(ft_list) >= meanrange:
    mean_data.data = np.average(ft_list[-meanrange:], axis=0)
  else:
    mean_data.data = c_ft
  
  pubraw.publish(raw_data)
  pubmean.publish(mean_data)
  time.sleep(0.01)

rospy.spin()
