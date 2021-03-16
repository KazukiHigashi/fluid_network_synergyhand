#!/usr/bin/env python
## coding: UTF-8

import rospy
from std_msgs.msg import Int8

iotopin = [4, 17, 18, 22, 23, 24, 27]
mode = [[[0, 1, 2, 3, 4],[]], # mode0
        [[0],[1, 2, 3, 4]], # mode1
        [[1],[0, 2, 3, 4]], # mode2 
        [[3, 4], [0, 1, 2]], # mode3
        [[0, 2, 3, 4], [1]], # mode4
        [[3], [0, 1, 2, 4]], # mode5
        [[4], [0, 1, 2, 3]], # mode6
        [[0, 2, 3, 4], [1]], # mode7
        [[0, 2, 3], [1, 4]] # mode8
        ]

for pin in iotopin:
  GPIO.setup(pin, GPIO.OUT)
  GPIO.output(pin, True)

def NetworkController(message):
  rospy.loginfo("change to mode[%d]", message.data)

  # Open Pins
  for openvalve in mode[message.data][0]:
    GPIO.output(iotopin[openvalve], False)
  
  for closevalve in mode[message.data][1]:
    GPIO.output(iotopin[closevalve], True)

rospy.init_node("network_mode_controller")
sub = rospy.Subscriber("send_mode", Int8, NetworkController)
rospy.spin()
  

