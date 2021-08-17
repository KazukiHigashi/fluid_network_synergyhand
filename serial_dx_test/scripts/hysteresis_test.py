#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, time
import struct
from dxlib import * 
from dx_comm import MX64Control

args = sys.argv
start = int(args[1])
goal = int(args[2])

motor = MX64Control(b"/dev/ttyS0", 57600, [1, 2], mode="extended_position")

time.sleep(2)

for i in range(5):
  motor.setPosition(start, 1)
  motor.setPosition(start, 2)

  time.sleep(10)

  motor.setPosition(goal, 1)
  motor.setPosition(goal, 2)

  time.sleep(10)

  
