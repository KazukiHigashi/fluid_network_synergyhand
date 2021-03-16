#!/usr/bin/env python
## coding: UTF-8

import spidev
import time
import RPi.GPIO as GPIO
import struct
import binascii
import rospy
from std_msgs.msg import Float32

def float2hex(f):
  return struct.unpack('>I', struct.pack('>f', f))[0]

def hex2float(s):
  # print([hex(i)[2:] for i in s])
  strdhex = "".join(["{:02x}".format(i) for i in s[::-1]])
  # print(strdhex)
  return struct.unpack('>f', binascii.unhexlify(strdhex))[0]

class MtlmotorSpi:
  def __init__(self, bus=0, device=0):
    self.spi = spidev.SpiDev()
    self.spi.open(bus, device)

    self.spi.mode = 3
    self.spi.max_speed_hz = 1000000

    self.s_data = [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00]
    self.r_data = [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
    
    self.motor_current = 0
    self.encoder_pulse = 0

  def fill_checksum(self):
    sm = 0x00
    for i in range(5):
      tmp = (self.s_data[2*i+1]<<8) + self.s_data[2*i]
      sm = sm + tmp
    sm = sm & 0xffff
    if not sm & 0x8000:
      sm = sm + 0x8000
    self.s_data[10] = sm&0xff
    self.s_data[11] = sm>>8

  def send(self, current):
    # Fill current data
    hexed = float2hex(current)
    for i in range(3, 7):
      self.s_data[i] = hexed & 0xff
      hexed = hexed >> 8
    # Fill checksum
    self.fill_checksum()
    # Send data
    self.r_data = self.spi.xfer3(self.s_data)
    self.encoder_pulse = hex2float(list(self.r_data[1:5]))
    self.motor_current = hex2float(list(self.r_data[5:9]))

  def __del__(self):
    self.spi.close()

def Motor1Callback(message):
  ddmotor1.send(float(-message.data))

def Motor2Callback(message):
  rospy.loginfo("got current:[%f]", message.data)
  ddmotor2.send(float(-message.data))

ddmotor1 = MtlmotorSpi(bus=0, device=0)
ddmotor2 = MtlmotorSpi(bus=0, device=1)

rospy.init_node("spi_comm")
sub = rospy.Subscriber("current_controller1", Float32, Motor1Callback)
sub = rospy.Subscriber("current_controller2", Float32, Motor2Callback)
rospy.spin()
