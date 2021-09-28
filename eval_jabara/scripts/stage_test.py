#!/usr/bin/env python
## coding: UTF-8

import time
import sys

from stage import StageControl


comport = sys.argv[1]
axis = "THETA_STAGE"

t_stage = StageControl(comport=comport, baudrate=9600)

t_stage.absolute_move(axis=axis, mm=10, speed=10)


