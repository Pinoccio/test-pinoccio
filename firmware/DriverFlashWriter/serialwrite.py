import serial
import time
import os
import binascii
import sys

filename = "scout-trunc.hex"
	
print "Connecting to serial"
ser = serial.Serial("/dev/tty.usbmodemfa131", 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, 5)

fp = open(filename)
size = os.path.getsize(filename)
print "Opened following hex file for flashing: ", fp.name, "(", size, "bytes )"
position = 0;

print "Waiting for Pinoccio:"
time.sleep(3)
ret = ""
running = True

while ser.inWaiting() > 0:
	ret += ser.read(1)
print ret

while (running and position < size / 3):

	hex = fp.read(2)
	val = int(hex, 16)
	print position, ") sent", val, "- 0x", hex
	ser.write(chr(val))
	fp.read(1)

	position+=1	

	if (position % 32 == 0):

		ret = ser.read(2)
		if (ret != "OK"):
			print "FAIL: no response from Scout"
			ret += ser.read(300)
			print ret
			running = False
		else:
			print ret
		
		ser.flushInput()
		
	if (position >= size):
		print "Done"
		running = False