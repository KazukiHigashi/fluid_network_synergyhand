#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 1軸へ角度と遷移時間を同時指令

import sys, time
import struct
from dxlib import *   # dxlibをインポート
from dx_comm import MX64Control

motor = MX64Control(b"/dev/ttyS0", 57600, [1, 2], mode="extended_position")

motor.setPosition(0, 1)
motor.setPosition(0, 2)

time.sleep(3)

