#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 1軸へ角度と遷移時間を同時指令

import sys, time
import struct
from dxlib import *   # dxlibをインポート

COMPort = b'/dev/ttyS0'  # 任意のCOMポート名に修正の事
Baudrate = 57600          # Dynamixelのボーレートと合わせる事
TargetID = 1              # DynamixelのIDと合わせる事

#---------------------------------------------
# (1) ポートを開いてdevを取得(必須)
dev = DX_OpenPort(COMPort, Baudrate)
if dev != None:
  # (2) 指定IDのモデル情報を取得(必須)
  if DXL_GetModelInfo(dev, TargetID).contents.devtype != devtNONE:
    # (3) 取得されたモデル情報からモデル名を表示(必須ではない)
    print (DXL_GetModelInfo(dev, TargetID).contents.name)

   
    DX_WriteWordData(dev, TargetID, 8, 4095, None) 
    DX_WriteWordData(dev, TargetID, 6, 4095, None) 

    DX_WriteWordData(dev, TargetID, 32, 1023, None)
    
    DX_WriteWordData(dev, TargetID, 30, -3000, None) 
    time.sleep(3)

    angle = (c_ushort*1)()

    print DX_ReadWordData(dev, TargetID, 36, angle, None)
    
    b = bin(*angle)
    # unsigned to signed
    sgn = int(b[3:],2) - int(int(b[2]) << (len(b)-3))
    print "{:4d}".format(sgn)
    
    # (7) トルクイネーブルディスエーブル(必要に応じて)
    DXL_SetTorqueEnable(dev, TargetID, False)
    # (8) ポートを閉じる(必須)
    DX_ClosePort(dev)
  else:
    print('Device information could not be acquired.')
else:
  print('Could not open COM port.')
