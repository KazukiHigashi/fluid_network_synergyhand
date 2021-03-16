import spidev
import time
import RPi.GPIO as GPIO
import struct
import binascii

spi1 = spidev.SpiDev()
spi2 = spidev.SpiDev()

spi1.open(0, 0)
spi2.open(0, 1)


spi1.mode = 3
spi1.max_speed_hz = 1000000

spi2.mode = 3
spi2.max_speed_hz = 1000000

to_send = [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x00, 0x00, 0x00, 0x00]

def float2hex(f: float) -> str:
  return struct.unpack('>I', struct.pack('>f', f))[0]

def calc_checksum(data):
  sm = 0x00
  for i in range(5):
    tmp = (data[2*i+1]<<8) + data[2*i]
    sm = sm + tmp
  sm = sm & 0xffff
  if not sm & 0x8000:
    sm = sm + 0x8000
  return sm&0xff, sm>>8

def hex2float(s: list) -> float:
  print([hex(i)[2:] for i in s])
  strdhex = "".join(["{:02x}".format(i) for i in s[::-1]])
  print(strdhex)
  return struct.unpack('>f', binascii.unhexlify(strdhex))[0]

curr = 0.0
dc = 0.05
inc = True
try:
  while(1):
    if inc == True:
      curr = curr + dc
      if curr >= 3:
        inc = False
    else:
      curr = curr - dc
      if curr <= -3:
        inc = True
    
    print(curr)

    ii = float2hex(curr)
    for i in range(3, 7):
      to_send[i] = ii & 0xff
      ii = ii >> 8
 
    res = calc_checksum(to_send)
    to_send[10] = res[0]
    to_send[11] = res[1]

    r_data = spi1.xfer3(to_send)
    spi2.xfer3(to_send)

    pulse_pos = hex2float(list(r_data[1:5]))
    motor_curr = hex2float(list(r_data[5:9]))

    print("received data: ", [hex(i) for i in r_data])
    print("pulse: ", pulse_pos)
    print("motor_curr: ", motor_curr)    
    print("\n")

    time.sleep(0.5)
except KeyboardInterrupt:
  spi1.close()
  spi2.xfer3(to_send)
  print("!FINISH!")

