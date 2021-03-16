#!/usr/bin/env python
## coding: UTF-8

import spidev
import time
import RPi.GPIO as GPIO
import struct
import binascii
import rospy
from std_msgs.msg import Int16MultiArray, Int8, Int16, Float32

rospy.init_node("joystick_controller")

njoysw = True
npushsw = True
nmode = 0
nkeep = False

mode_transition = [0, 2, 3, 4]

pot_num = 7
joy_num = 8

pc = None # previous current


def GetPotCb(message):
  global njoysw, npushsw, nmode, pubcurr, pubmode, pot_num, joy_num, nkeep, pc
  joyx = message.data[pot_num]
  joysw = message.data[pot_num + 2] # active low
  pushsw = message.data[pot_num + 3] # active low

  curr = Float32()
  curr.data = 7.5*(joyx-512)/1024.0

  if npushsw and pushsw == 0:
    npushsw = False
    nkeep = not nkeep # toggle boolean
    pc = curr.data
  elif pushsw == 1:
    npushsw = True

  if nkeep:
    # Change the valve mode to Keep Grasp Function
    if nmode != 5:
      mode = Int16()
      mode.data = 5
      pubmode.publish(mode)
      nmode = 5
  
    # The index finger is controlled dynamically.
    pubcurr1.publish(curr) 
    
    curr.data = pc + (3.5 - pc)/30.0 # target current is 3.5A
    pc = curr.data
    pubcurr2.publish(curr)
  else:
    if njoysw and joysw == 0:
      njoysw = False
      nmode += 1
      if nmode > 3:
        nmode = 0
      mode = Int16()
      mode.data = mode_transition[nmode]
      pubmode.publish(mode)
    elif joysw == 1:
      njoysw = True
    pubcurr1.publish(curr)
    pubcurr2.publish(curr)

subpot = rospy.Subscriber("send_pot", Int16MultiArray, GetPotCb)
pubcurr1 = rospy.Publisher("current_controller1", Float32, queue_size=10)
pubcurr2 = rospy.Publisher("current_controller2", Float32, queue_size=10)
pubmode = rospy.Publisher("mode_setter", Int16, queue_size=10)

rospy.spin()
