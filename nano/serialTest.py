
import serial
from time import sleep
import sys
 
COM_PORT = "COM5"
BAUD_RATES = 115200
ser = serial.Serial(COM_PORT, BAUD_RATES)
 
try:
    while True:
        #send command
        header = 0xFE
        cmd = 0x01
        argNum = 2
        forward = 128
        turn = 128
        msg = [header,cmd,argNum,forward,turn]
        #compute checksum
        checksum = 0
        for ch in msg:
            checksum += ch
        checksum = checksum%256
        msg.append(checksum)
        ser.write(bytearray(msg))

        #receive status
        while ser.in_waiting:
            msg = ser.readline().decode()  # 接收回應訊息並解碼
            print('控制板狀態：', msg)

        sleep(1)

except KeyboardInterrupt:
    ser.close()