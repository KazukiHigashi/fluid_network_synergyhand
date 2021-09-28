#!/usr/bin/env python
## coding: UTF-8

import time
import sys
import rospy
from std_msgs.msg import Float32

from stage_py2 import StageControl

comport = sys.argv[1]
axis = "THETA_STAGE"

t_stage = StageControl(comport=comport, baudrate=9600)

t_stage.absolute_move(axis=axis, mm=90, speed=10)


