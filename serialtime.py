import serial
import sys
r1 = 8200 # resister 1
r2 = 2000 # resister 2
comPort = sys.argv[1]
ser = serial.Serial(comPort, 9600, timeout=1)

while True: 
  line = str(ser.readline())   
  if "AnalogValue:" in line: 
   line = line.strip("b'AnalogValue:")
   vcc = line.strip("\\r\\n'")
   vcc = int(vcc)
   vcc = vcc * (0.000244)
   vout = vcc - ((vcc*r1)/(r1+r2))
   print(vout)
  elif "DigitalValue:" in line:
    line = line.strip("b'DigitalValue:")
    line = line.strip("\\r\\n'")
    line = int(line)
    if line == 0:
      print("kill")
    else:
        print("arm")
