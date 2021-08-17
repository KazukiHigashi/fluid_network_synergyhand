#!/usr/bin/env python
## coding: UTF-8

import time
import sys
from dxlib import *

comport = b'/dev/ttyS0'
baudrate = 57600
target_id = 1

dev = DX_OpenPort(comport, baudrate)
if dev != None:
  # (2) 指定IDのモデル情報を取得(必須)
  if DXL_GetModelInfo(dev, target_id).contents.devtype != devtNONE:
    # (3) 取得されたモデル情報からモデル名を表示(必須ではない)
    print (DXL_GetModelInfo(dev, target_id).contents.name)

    cid = (c_ubyte*1)()
    DX_ReadByteData(dev, target_id, 3, cid, None)

    print "Current ID"
    print int(*cid)

    new_id = 2
    DX_WriteByteData(dev, target_id, 3, new_id, None)

    DX_ReadByteData(dev, new_id, 3, cid, None)
    print "New ID"
    print int(*cid)

 
  else:
    print('Device information could not be acquired.')
else:
  print('Could not open COM port.')
