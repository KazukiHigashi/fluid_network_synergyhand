import serial

ser = serial.Serial('/dev/ttyS0', '9600', timeout=0.1)

ser.write('Hello, World!')
print repr(ser.readline())
ser.close()
