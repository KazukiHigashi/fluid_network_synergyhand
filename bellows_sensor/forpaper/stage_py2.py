#!/usr/bin python
# -*- coding: utf-8 -*-

import struct
import serial

class StageControl:
  def __init__(self, comport='/dev/ttyUSB0', baudrate=9600):
    self.ser = serial.Serial(comport, baudrate, timeout=0.1)

  def send(self, buf):
      # print(buf)
      while True:
          if self.ser.out_waiting == 0:
              break
      for b in buf:
          # print(b)
          # a = struct.pack("B", b)
          # print(a)
          self.ser.write(b)
          # print(self.ser.read(10))
      self.ser.flush()

  def absolute_move(self, axis, mm, speed):
      if speed > 100:
          speed = 100
      elif speed < 10:
          speed = 10
      
      if axis == 'X_STAGE':
          axis_num = 1
          pulse = int( abs(mm)*1000)
          
          max_speed = int(speed * 250)
          min_speed = int(max_speed/10)
          te = int(2*10000/speed)
          
          if te>1000:
              te = 1000
          elif te<200:
              te = 200         

          pre_cmd = "D:1S".encode() + str(min_speed).encode() + "F".encode() + str(max_speed).encode() + "R".encode() + str(te).encode() + "\r\n".encode()

      elif axis == 'THETA_STAGE':
          axis_num = 2
          pulse = int( abs(mm) * 400)

          max_speed = int(speed * 120)
          min_speed = int(max_speed/10)
          te = int(2*10000/speed)

          if te>1000:
              te = 1000

          pre_cmd = "D:2S".encode() + str(min_speed).encode() + "F".encode() + str(max_speed).encode() + "R".encode() + str(te).encode() + "\r\n".encode()

      else:
          axis_num = 0
          pulse = 0
          print("axis_num_error")

      if mm > 0:
          cmd = "A:".encode() + str(axis_num).encode() + "+P".encode() + str(pulse).encode() + "\r\n".encode() + "G:".encode() + "\r\n".encode()

      else:
          cmd = "A:".encode() + str(axis_num).encode() + "-P".encode() + str(pulse).encode() + "\r\n".encode() + "G:".encode() + "\r\n".encode()

      self.send(pre_cmd)
      self.send(cmd)

  def relative_move(self, axis, mm, speed):
      if speed > 100:
          speed = 100
      elif speed < 10:
          speed = 10

      if axis == 'X_STAGE':
          axis_num = 1
          pulse = int( abs(mm)*1000)

          max_speed = int(speed * 250)
          min_speed = int(max_speed/10)
          te = int(2*10000/speed)

          if te>1000:
              te = 1000
          elif te<200:
              te = 200

          pre_cmd = "D:1S".encode() + str(min_speed).encode() \
                    + "F".encode() + str(max_speed).encode() \
                    + "R".encode() + str(te).encode() + "\r\n".encode()

      elif axis == 'THETA_STAGE':
          axis_num = 2
          pulse = int( abs(mm) * 400)

          max_speed = int(speed * 120)
          min_speed = int(max_speed/10)
          te = int(2*10000/speed)

          if te>1000:
              te = 1000

          pre_cmd = "D:2S".encode() + str(min_speed).encode() \
                    + "F".encode() + str(max_speed).encode() \
                    + "R".encode() + str(te).encode() + "\r\n".encode()

      else:
          axis_num = 0
          pulse = 0
          print("axis_num_error")

      if mm > 0:
          cmd = "M:".encode() + str(axis_num).encode() + "+P".encode() \
                + str(pulse).encode() + "\r\n".encode() + "G:".encode() + "\r\n".encode()

      else:
          cmd = "M:".encode() + str(axis_num).encode() + "-P".encode() \
                + str(pulse).encode() + "\r\n".encode() + "G:".encode() + "\r\n".encode()

      #print(pre_cmd)
      self.send(pre_cmd)
      self.send(cmd)

  def set_zero(self, axis):

      if axis == 'X_STAGE':
          axis_num = 1
      elif axis == 'THETA_STAGE':
          axis_num = 2
      else:
          axis_num = 0
          pulse = 0
          print("axis_num_error")

      cmd = "R:".encode() + str(axis_num).encode() + "\r\n".encode()
      self.send(cmd)

