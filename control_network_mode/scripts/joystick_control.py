#!/usr/bin/env python
## coding: UTF-8

import spidev
import time
import rospy
import numpy as np
from std_msgs.msg import Int16MultiArray, Int8, Int16, Float32

rospy.init_node("joystick_controller")

njoysw = True
npushsw = True
nmode = 0
nkeep = False
keepdone = False

mode_transition = [0, 2, 3, 4]

pot_num = 7
joy_num = 8

queue = [] 

def GetPotCb(message):
  global njoysw, npushsw, nmode, pubcurr, pubmode, pot_num, joy_num, nkeep, queue, keepdone
  posture = message.data[:pot_num]
  joyx = message.data[pot_num]
  joysw = message.data[pot_num + 2] # active low
  pushsw = message.data[pot_num + 3] # active low

  curr = Float32()
  # curr.data = 7.5*(joyx-512)/1024.0 # For Mtl DD motors
  
  # For Dynamixel motors
  # -23.04 [mA] ~ +23.04[mA]
  # Soft Threshold
  # 500 ~ 524
  if joyx >= 500 and joyx <= 524:
    curr.data = 1024
  elif joyx < 512:
    joyx = joyx + 12
    curr.data = (512-joyx)*0.3
  else:
    joyx = joyx - 12
    curr.data = 1024 + (joyx-512)*0.3

  if npushsw and pushsw == 0:
    npushsw = False
    nkeep = not nkeep # toggle boolean
    keepdone = False
    pc = curr.data
  elif pushsw == 1:
    npushsw = True

  if nkeep:
    # Change the valve mode to Keep Grasp Function
    if nmode != 5:
      mode = Int16()
      mode.data = 6 # 5 or 6
      pubmode.publish(mode)
      nmode = 6
    
    # The index finger is controlled dynamically.
    pubcurr1.publish(curr)
    
    # Watch angles of the middle finger
    if len(queue) > 50:
      max_err = np.max(np.linalg.norm(np.array(queue) - np.array(posture[:4]), axis=1))
      print max_err

      if max_err < 50:
        keepdone = True
        curr.data = 1024
        pubcurr2.publish(curr)
        queue = []
        print "!!!"
      else:
        curr.data = 1080
        pubcurr2.publish(curr)
        queue.append(posture[:4])
        if len(queue) > 60:
          queue.pop(0)
    elif not keepdone:
      queue.append(posture[:4])
      curr.data = 1080
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
