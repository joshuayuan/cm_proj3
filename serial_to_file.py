import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)
fname = raw_input("file name: ")
f = open(fname, "w")

count = 0
while True:
    line = ser.readline()
    f.write(line.strip())
    print(line),
    print(count)
    count += 1
    time.sleep(0.1);

