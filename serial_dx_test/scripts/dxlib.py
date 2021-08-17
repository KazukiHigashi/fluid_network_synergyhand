#
# Dynamixel 1.0 Library v4.4
#                                       Last Edit '21 01/22
#   Copyright (c) 2005, 2021 BestTechnology CO.,LTD.
#

from ctypes import *
from sys import maxsize
from sysconfig import get_platform

__version__ = "4.4.0.0"

if ('mingw' in get_platform()) or ('win' in get_platform()):
  if (maxsize > 2**32) == True:
    _dxlib = WinDLL('./dxlib_x64.dll')
  else:
    _dxlib = WinDLL('./dxlib_x32.dll')
else:
  _dxlib = CDLL('/home/ubuntu/workspace/src/fluid_network_synergyhand/serial_dx_test/scripts/dxlib.so.4.4')

_dxlib.DX_OpenPort.argtypes = [c_char_p, c_uint32]
_dxlib.DX_OpenPort.restype  = c_void_p
def DX_OpenPort(name, baud):
  return _dxlib.DX_OpenPort(name,baud)

_dxlib.DX_ClosePort.argtypes = [c_void_p]
_dxlib.DX_ClosePort.restype   = c_bool
def DX_ClosePort(dvid):
  return _dxlib.DX_ClosePort(dvid)

_dxlib.DX_SetBaudrate.argtypes = [c_void_p, c_ulong]
_dxlib.DX_SetBaudrate.restype   = c_bool
def DX_SetBaudrate(dvid,baud):
  return _dxlib.DX_SetBaudrate(dvid,baud)

_dxlib.DX_Active.argtypes = [c_void_p]
_dxlib.DX_Active.restype   = c_bool
def DX_Active(dvid):
  return _dxlib.DX_Active(dvid)

_dxlib.DX_SetTimeOutOffset.argtypes = [c_void_p, c_uint32]
_dxlib.DX_SetTimeOutOffset.restype   =  (None)
def DX_SetTimeOutOffset(dvid,offsettime):
  return _dxlib.DX_SetTimeOutOffset(dvid,offsettime)

_dxlib.GetQueryPerformanceCounter.argtypes = (None)
_dxlib.GetQueryPerformanceCounter.restype = c_double
def GetQueryPerformanceCounter():
  return _dxlib.GetQueryPerformanceCounter()

_dxlib.DX_TxPacket.argtypes = [c_void_p, c_uint8, c_uint8, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dxlib.DX_TxPacket.restype  = c_bool
def DX_TxPacket(dvid,id,inst,param,len,err):
  return _dxlib.DX_TxPacket(dvid,id,inst,param,len,err)

_dxlib.DX_RxPacket.argtypes = [c_void_p, POINTER(c_uint8), c_uint32, POINTER(c_uint32), c_uint32, POINTER(c_uint16)]
_dxlib.DX_RxPacket.restype  = c_bool
def DX_RxPacket(dvid,rdata,rdatasize,rlen,timeout,err):
  return _dxlib.DX_RxPacket(dvid,rdata,rdatasize,rlen,timeout,err)

_dxlib.DX_ReadByteData.argtypes = [c_void_p, c_uint8, c_uint16, POINTER(c_uint8), POINTER(c_uint16)]
_dxlib.DX_ReadByteData.restype  = c_bool
def DX_ReadByteData(dvid,id,addr,rdat,err):
  return _dxlib.DX_ReadByteData(dvid,id,addr,rdat,err)

_dxlib.DX_WriteByteData.argtypes = [c_void_p, c_uint8, c_uint16, c_uint8, POINTER(c_uint16)]
_dxlib.DX_WriteByteData.restype  = c_bool
def DX_WriteByteData(dvid,id,addr,wdat,err):
  return _dxlib.DX_WriteByteData(dvid,id,addr,wdat,err)

_dxlib.DX_ReadWordData.argtypes = [c_void_p, c_uint8, c_uint16, POINTER(c_uint16), POINTER(c_uint16)]
_dxlib.DX_ReadWordData.restype  = c_bool
def DX_ReadWordData(dvid,id,addr,rdat,err):
  return _dxlib.DX_ReadWordData(dvid,id,addr,rdat,err)

_dxlib.DX_WriteWordData.argtypes = [c_void_p, c_uint8, c_uint16, c_uint16, POINTER(c_uint16)]
_dxlib.DX_WriteWordData.restype  = c_bool
def DX_WriteWordData(dvid,id,addr,wdat,err):
  return _dxlib.DX_WriteWordData(dvid,id,addr,wdat,err)

_dxlib.DX_ReadLongData.argtypes = [c_void_p, c_uint8, c_uint16, POINTER(c_uint32), POINTER(c_uint16)]
_dxlib.DX_ReadLongData.restype  = c_bool
def DX_ReadLongData(dvid,id,addr,rdat,err):
  return _dxlib.DX_ReadLongData(dvid,id,addr,rdat,err)

_dxlib.DX_WriteLongData.argtypes = [c_void_p, c_uint8, c_uint16, c_uint32, POINTER(c_uint16)]
_dxlib.DX_WriteLongData.restype  = c_bool
def DX_WriteLongData(dvid,id,addr,wdat,err):
  return _dxlib.DX_WriteLongData(dvid,id,addr,wdat,err)

_dxlib.DX_ReadBlockData.argtypes = [c_void_p, c_uint8, c_ushort, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dxlib.DX_ReadBlockData.restype  = c_bool
def DX_ReadBlockData(dvid,id,addr,rdat,len,err):
  return _dxlib.DX_ReadBlockData(dvid,id,addr,rdat,len,err)

_dxlib.DX_WriteBlockData.argtypes = [c_void_p, c_uint8, c_uint16, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dxlib.DX_WriteBlockData.restype  = c_bool
def DX_WriteBlockData(dvid,id,addr,dat,len,err):
  return _dxlib.DX_WriteBlockData(dvid,id,addr,dat,len,err)

_dxlib.DX_Ping.argtypes = [c_void_p, c_uint8, POINTER(c_uint16)]
_dxlib.DX_Ping.restype  = c_bool
def DX_Ping(dvid,id,err):
  return _dxlib.DX_Ping(dvid,id,err)

class TDxAlarmStatus(Structure):
  _pack_ = 1
  _fields_ = [("id", c_uint8),("Status", c_uint16)]

_dxlib.DX_Ping2.argtypes = [c_void_p, POINTER(c_int32), POINTER(TDxAlarmStatus), POINTER(c_uint16)]
_dxlib.DX_Ping2.restype  = c_bool
def DX_Ping2(dvid,num,AlarmStatus,err):
  return _dxlib.DX_Ping2(dvid,num,AlarmStatus,err)

_dxlib.DX_WriteSyncData.argtypes = [c_void_p, POINTER(c_uint8), c_uint32, POINTER(c_uint16)]
_dxlib.DX_WriteSyncData.restype  = c_bool
def DX_WriteSyncData(dvid,dat,size,err):
  return _dxlib.DX_WriteSyncData(dvid,dat,size,err)

_dxlib.DX_Reset.argtypes = [c_void_p, c_uint8, c_void_p]
_dxlib.DX_Reset.restype  = c_bool
def DX_Reset(dvid,id,err):
  return _dxlib.DX_Reset(dvid,id,err)

(devtNONE, devtDX, devtAX, devtRX, devtEX, devtMX, devtXL320, devtPRO, devtX) = range(9)

class TMaxMin_int32(Structure):
  _pack_ = 1
  _fields_ = [("max", c_int32),("min", c_int32)]
class TMaxMin_dbl(Structure):
  _pack_ = 1
  _fields_ = [("max", c_double),("min", c_double)]

class TDXL_ModelInfo(Structure):
  _pack_ = 1
  _fields_ = [
    ("modelno",       c_uint16),
    ("name",          c_char * 16),
    ("devtype",       c_uint32),
    ("positionlimit", TMaxMin_int32),
    ("anglelimit",    TMaxMin_dbl),
    ("velocitylimit", TMaxMin_int32),
    ("pwmlimit",      TMaxMin_int32),
    ("velocityratio",      c_double),
    ("currentratio",  c_double),
    ("pwmratio",      c_double)]

class TAngleVelocity(Structure):
  _fields_ = [
    ("angle", c_double),
    ("velocity", c_double)]

# DXL LED
#----------------------------
_dxlib.DXL_SetLED.argtypes = [c_void_p, c_uint8, c_bool]
_dxlib.DXL_SetLED.restype  = c_bool
def DXL_SetLED(dvid,id,en):
  return _dxlib.DXL_SetLED(dvid,id,en)

# DXL TorqueEnable
#----------------------------
_dxlib.DXL_SetTorqueEnable.argtypes = [c_void_p, c_uint8, c_bool]
_dxlib.DXL_SetTorqueEnable.restype  = c_bool
def DXL_SetTorqueEnable(dvid,id,en):
  return _dxlib.DXL_SetTorqueEnable(dvid,id,en)

_dxlib.DXL_SetTorqueEnables.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_bool), c_int]
_dxlib.DXL_SetTorqueEnables.restype  = c_bool
def DXL_SetTorqueEnables(dvid,ids,en,num):
  return _dxlib.DXL_SetTorqueEnables(dvid,ids,en,num)

_dxlib.DXL_SetTorqueEnablesEquival.argtypes = [c_void_p, POINTER(c_uint8), c_int, c_bool]
_dxlib.DXL_SetTorqueEnablesEquival.restype  = c_bool
def DXL_SetTorqueEnablesEquival(dvid,ids,num,en):
  return _dxlib.DXL_SetTorqueEnablesEquival(dvid,ids,num,en)

_dxlib.DXL_GetTorqueEnable.argtypes = [c_void_p, c_uint8, POINTER(c_bool)]
_dxlib.DXL_GetTorqueEnable.restype  = c_bool
def DXL_GetPresentAngle(dvid,id,en):
  return _dxlib.DXL_GetTorqueEnable(dvid,id,en)

_dxlib.DXL_GetTorqueEnables.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_bool), c_int]
_dxlib.DXL_GetTorqueEnables.restype  = c_bool
def DXL_GetTorqueEnables(dvid,ids,en,num):
  return _dxlib.DXL_GetTorqueEnables(dvid,ids,en,num)

# DXL Angle
#----------------------------
_dxlib.DXL_SetGoalAngle.argtypes = [c_void_p, c_uint8, c_double]
_dxlib.DXL_SetGoalAngle.restype  = c_bool
def DXL_SetGoalAngle(dvid,id,angle):
  return _dxlib.DXL_SetGoalAngle(dvid,id,angle)

_dxlib.DXL_SetGoalAngles.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dxlib.DXL_SetGoalAngles.restype  = c_bool
def DXL_SetGoalAngles(dvid,ids,angles,num):
  return _dxlib.DXL_SetGoalAngles(dvid,ids,angles,num)

_dxlib.DXL_GetPresentAngle.argtypes = [c_void_p, c_uint8, POINTER(c_double)]
_dxlib.DXL_GetPresentAngle.restype  = c_bool
def DXL_GetPresentAngle(dvid,id,angle):
  return _dxlib.DXL_GetPresentAngle(dvid,id,angle)

_dxlib.DXL_GetPresentAngles.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dxlib.DXL_GetPresentAngles.restype  = c_bool
def DXL_GetPresentAngles(dvid,ids,angles,num):
  return _dxlib.DXL_GetPresentAngles(dvid,ids,angles,num)

# DXL Standstill
#----------------------------
_dxlib.DXL_StandStillAngle.argtypes = [c_void_p, c_uint8]
_dxlib.DXL_StandStillAngle.restype  = c_bool
def DXL_StandStillAngle(dvid,id):
  return _dxlib.DXL_StandStillAngle(dvid,id)

_dxlib.DXL_StandStillAngles.argtypes = [c_void_p, POINTER(c_uint8), c_int]
_dxlib.DXL_StandStillAngles.restype  = c_bool
def DXL_StandStillAngles(dvid,ids,num):
  return _dxlib.DXL_StandStillAngles(dvid,ids,num)

# DXL Velocity
#----------------------------
_dxlib.DXL_SetGoalVelocity.argtypes = [c_void_p, c_uint8, c_double]
_dxlib.DXL_SetGoalVelocity.restype  = c_bool
def DXL_SetGoalVelocity(dvid,id,velocity):
  return _dxlib.DXL_SetGoalVelocity(dvid,id,velocity)

_dxlib.DXL_SetGoalVelocities.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dxlib.DXL_SetGoalVelocities.restype  = c_bool
def DXL_SetGoalVelocities(dvid,ids,velocities,num):
  return _dxlib.DXL_SetGoalVelocities(dvid,ids,velocities,num)

_dxlib.DXL_GetPresentVelocity.argtypes = [c_void_p, c_uint8, POINTER(c_double)]
_dxlib.DXL_GetPresentVelocity.restype  = c_bool
def DXL_GetPresentVelocity(dvid,id,velocity):
  return _dxlib.DXL_GetPresentVelocity(dvid,id,velocity)

_dxlib.DXL_GetPresentVelocities.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dxlib.DXL_GetPresentVelocities.restype  = c_bool
def DXL_GetPresentVelocities(dvid,ids,velocities,num):
  return _dxlib.DXL_GetPresentVelocities(dvid,ids,velocities,num)

# DXL Angle & Velocity
#----------------------------
_dxlib.DXL_SetGoalAngleAndVelocity.argtypes = [c_void_p, c_uint8, c_double, c_double]
_dxlib.DXL_SetGoalAngleAndVelocity.restype  = c_bool
def DXL_SetGoalAngleAndVelocity(dvid,id,angle,velocity):
  return _dxlib.DXL_SetGoalAngleAndVelocity(dvid,id,angle,velocity)

_dxlib.DXL_SetGoalAnglesAndVelocities.argtypes = [c_void_p, POINTER(c_uint8), POINTER(TAngleVelocity), c_int]
_dxlib.DXL_SetGoalAnglesAndVelocities.restype  = c_bool
def DXL_SetGoalAnglesAndVelocities(dvid,ids,anglevelocity,num):
  return _dxlib.DXL_SetGoalAnglesAndVelocities(dvid,ids,anglevelocity,num)

# DXL Angle & Time
#----------------------------
_dxlib.DXL_SetGoalAngleAndTime.argtypes = [c_void_p, c_uint8, c_double, c_double]
_dxlib.DXL_SetGoalAngleAndTime.restype  = c_bool
def DXL_SetGoalAngleAndTime(dvid,id,angle,sec):
  return _dxlib.DXL_SetGoalAngleAndTime(dvid,id,angle,sec)

_dxlib.DXL_SetGoalAnglesAndTime.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int, c_double]
_dxlib.DXL_SetGoalAnglesAndTime.restype  = c_bool
def DXL_SetGoalAnglesAndTime(dvid,ids,angles,num,sec):
  return _dxlib.DXL_SetGoalAnglesAndTime(dvid,ids,angles,num,sec)

# DXL Current
#----------------------------
_dxlib.DXL_SetGoalCurrent.argtypes = [c_void_p, c_uint8, c_double]
_dxlib.DXL_SetGoalCurrent.restype  = c_bool
def DXL_SetGoalCurrent(dvid,id,current):
  return _dxlib.DXL_SetGoalCurrent(dvid,id,current)

_dxlib.DXL_SetGoalCurrents.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dxlib.DXL_SetGoalCurrents.restype  = c_bool
def DXL_SetGoalCurrents(dvid,ids,currents,num):
  return _dxlib.DXL_SetGoalCurrents(dvid,ids,currents,num)

_dxlib.DXL_GetPresentCurrent.argtypes = [c_void_p, c_uint8, POINTER(c_double)]
_dxlib.DXL_GetPresentCurrent.restype  = c_bool
def DXL_GetPresentCurrent(dvid,id,current):
  return _dxlib.DXL_GetPresentCurrent(dvid,id,current)

_dxlib.DXL_GetPresentCurrents.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dxlib.DXL_GetPresentCurrents.restype  = c_bool
def DXL_GetPresentCurrents(dvid,ids,currents,num):
  return _dxlib.DXL_GetPresentCurrents(dvid,ids,currents,num)

# DXL PWM
#----------------------------
_dxlib.DXL_SetGoalPWM.argtypes = [c_void_p, c_uint8, c_double]
_dxlib.DXL_SetGoalPWM.restype  = c_bool
def DXL_SetGoalPWM(dvid,id,pwm):
  return _dxlib.DXL_SetGoalPWM(dvid,id,pwm)

_dxlib.DXL_SetGoalPWMs.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dxlib.DXL_SetGoalPWMs.restype  = c_bool
def DXL_SetGoalPWMs(dvid,ids,pwms,num):
  return _dxlib.DXL_SetGoalPWMs(dvid,ids,pwms,num)

_dxlib.DXL_GetPresentPWM.argtypes = [c_void_p, c_uint8, POINTER(c_double)]
_dxlib.DXL_GetPresentPWM.restype  = c_bool
def DXL_GetPresentPWM(dvid,id,pwm):
  return _dxlib.DXL_GetPresentPWM(dvid,id,pwm)

_dxlib.DXL_GetPresentPWMs.argtypes = [c_void_p, POINTER(c_uint8), POINTER(c_double), c_int]
_dxlib.DXL_GetPresentPWMs.restype  = c_bool
def DXL_GetPresentPWMs(dvid,ids,pwms,num):
  return _dxlib.DXL_GetPresentPWMs(dvid,ids,pwms,num)

# DXL OP Mode
#----------------------------
_dxlib.DXL_SetOperatingMode.argtypes = [c_void_p, c_uint8, c_uint8]
_dxlib.DXL_SetOperatingMode.restype  = c_bool
def DXL_SetOperatingMode(dvid,id,mode):
  return _dxlib.DXL_SetOperatingMode(dvid,id,mode)

_dxlib.DXL_SetOperatingModesEquival.argtypes = [c_void_p, POINTER(c_uint8), c_int, c_uint8]
_dxlib.DXL_SetOperatingModesEquival.restype  = c_bool
def DXL_SetOperatingModesEquival(dvid,ids,num,mode):
  return _dxlib.DXL_SetOperatingModesEquival(dvid,ids,num,mode)

_dxlib.DXL_GetOperatingMode.argtypes = [c_void_p, c_uint8, POINTER(c_uint8)]
_dxlib.DXL_GetOperatingMode.restype  = c_bool
def DXL_GetOperatingMode(dvid,id,mode):
  return _dxlib.DXL_GetOperatingMode(dvid,id,mode)

# DXL Error
#----------------------------
_dxlib.DXL_GetErrorCode.argtypes = [c_void_p, c_uint8]
_dxlib.DXL_GetErrorCode.restype  = c_uint16
def DXL_GetErrorCode(dvid,id):
  return _dxlib.DXL_GetErrorCode(dvid,id)

_dxlib.DXL_GetHWErrorCode.argtypes = [c_void_p, c_uint8, POINTER(c_uint8)]
_dxlib.DXL_GetHWErrorCode.restype  = c_bool
def DXL_GetHWErrorCode(dvid,id,hwerr):
  return _dxlib.DXL_GetHWErrorCode(dvid,id,hwerr)

# DXL Model Info
#----------------------------
_dxlib.DXL_GetModelInfo.argtypes = [c_void_p, c_uint8]
_dxlib.DXL_GetModelInfo.restype  = POINTER(TDXL_ModelInfo)
def DXL_GetModelInfo(dvid,id):
  return _dxlib.DXL_GetModelInfo(dvid,id)

_dxlib.DXL_ScanDevices.argtypes = [c_void_p]
_dxlib.DXL_ScanDevices.restype  = c_int
def DXL_ScanDevices(dvid):
  return _dxlib.DXL_ScanDevices(dvid)

_dxlib.DXL_PrintDevicesList.argtypes = [c_void_p]
_dxlib.DXL_PrintDevicesList.restype  = c_bool
def DXL_PrintDevicesList(f):
  return _dxlib.DXL_PrintDevicesList(f)

_dxlib.DXL_InitDevicesList.argtypes = (None)
_dxlib.DXL_InitDevicesList.restype  = (None)
def DXL_InitDevicesList():
  _dxlib.DXL_InitDevicesList()
