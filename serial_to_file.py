import serial
import time

port = raw_input("port: ")
ser = serial.Serial(port, 115200)
fname = raw_input("file name: ")
f = open(fname, "w")

count = 0
while True:
    line = ser.readline()
    f.write(line)
    print(line),
    print(count)
    count += 1
    time.sleep(0.1);

