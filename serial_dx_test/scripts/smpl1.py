#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 1軸のLED明滅

import sys, time
from dxlib import *   # dxlibをインポート

COMPort = b'/dev/ttyS0' # 任意のCOMポート名に修正の事
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

    # (4) LED点灯
    DXL_SetLED(dev, TargetID, True)
    # (5) 1秒待ち
    time.sleep(1)
    # (6) LED消灯
    DXL_SetLED(dev, TargetID, False)

    # (7) ポートを閉じる(必須)
    DX_ClosePort(dev)
  else:
    print('Device information could not be acquired.')
else:
  print('Could not open COM port.')
