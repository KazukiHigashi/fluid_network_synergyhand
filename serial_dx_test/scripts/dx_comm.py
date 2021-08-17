#!/usr/bin/env python
## coding: UTF-8

import time
import sys
from dxlib import *
import rospy
from std_msgs.msg import Float32

comport = b'/dev/ttyS0'
baudrate = 57600
target_id = 1

class MX64Control:
  def __init__(self, comport, baudrate, target_ids, mode="torque"):
    self.dev = DX_OpenPort(comport, baudrate)
    self.mode = mode
    if self.dev != None:
      for target_id in target_ids:
        # Get Motor ID
        if DXL_GetModelInfo(self.dev, target_id).contents.devtype != devtNONE:
          # Display a model name from its id.
          print (DXL_GetModelInfo(self.dev, target_id).contents.name)

          if self.mode == "torque":
            # Enable the torque mode
            DX_WriteByteData(self.dev, target_id, 70, 1, None)
            # Set a goal torque (1024 = 0[A])
            DX_WriteWordData(self.dev, target_id, 71, 1024, None)
          if self.mode == "extended_position":
            DX_WriteWordData(self.dev, target_id, 8, 4095, None) 
            DX_WriteWordData(self.dev, target_id, 6, 4095, None)
            # set speed: 0 ~ 1023 : 0 ~ 117.07 rpm
            DX_WriteWordData(self.dev, target_id, 32, 300, None)

        else:
          print('Device information could not be acquired.')
    else:
      print('Could not open COM port.')

  def getPosition(self, target_id):
    cp = (c_ushort*1)()
    DX_ReadWordData(self.dev, target_id, 36, cp, None)
    
    b = bin(*cp)
    # unsigned to signed
    sgn = int(b[3:],2) - int(int(b[2]) << (len(b)-3))
    return sgn

  def setPosition(self, position, target_id):
    assert self.mode == "extended_position"
    
    DX_WriteWordData(self.dev, target_id, 30, position, None)

  def setTorque(self, current, target_id):
    # 0    ~ 1023 : CCW
    # 1024 ~ 2047 : CW
    # Unit : 4.5 [mA]
    assert self.mode == "torque"
    
    print current
    DX_WriteWordData(self.dev, target_id, 71, current, None)
    
    cc = (c_ushort*1)()
    DX_ReadWordData(self.dev, target_id, 68, cc, None)

    return (int(*cc)-2048)*9.2115/4095.0

  def __del__(self):
    if self.mode == "torque":
      motor.setTorque(1024, 1)
      motor.setTorque(1024, 2)
    DX_ClosePort(self.dev)
  
