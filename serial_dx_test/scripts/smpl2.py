#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 1軸へ角度と角速度を同時指令

import sys, time
from dxlib import *   # dxlibをインポート

COMPort = b'\\\\.\\COM3'  # 任意のCOMポート名に修正の事
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

    # (4) DynamixelをJointモード=3に変更
    DXL_SetOperatingMode(dev, TargetID, 3)
    # (5) トルクイネーブル(必須)
    DXL_SetTorqueEnable(dev, TargetID, True)

    # (6) 角度[deg]と角速度を[deg/sec]を順次指令
    DXL_SetGoalAngleAndVelocity(dev, TargetID,  90.0, 180.0)
    time.sleep(0.5)
    DXL_SetGoalAngleAndVelocity(dev, TargetID,   0.0, 180.0)
    time.sleep(0.5)
    DXL_SetGoalAngleAndVelocity(dev, TargetID, -90.0, 180.0)
    time.sleep(0.5)
    DXL_SetGoalAngleAndVelocity(dev, TargetID,   0.0, 180.0)
    time.sleep(1)

    # (7) トルクイネーブルディスエーブル
    DXL_SetTorqueEnable(dev, TargetID, False)

    # (8) ポートを閉じる(必須)
    DX_ClosePort(dev)
  else:
    print('Device information could not be acquired.')
else:
  print('Could not open COM port.')
