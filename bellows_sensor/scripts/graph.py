#!/usr/bin/env python
## coding: UTF-8

import rospy
import time
import signal
import sys
import os
from std_msgs.msg import Float32MultiArray, Int16MultiArray, Float32, Int16

rospy.init_node("graph")
def potcb(message):
  pubpot.publish(message.data[0])
  pubbs.publish(message.data[1])


pubpot = rospy.Publisher("potentiometer", Int16, queue_size=10)
pubbs = rospy.Publisher("bellows_sensor", Int16, queue_size=10)
subpot = rospy.Subscriber("publish_pot_angle", Int16MultiArray, potcb)



rospy.spin()
