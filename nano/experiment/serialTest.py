
import serial
from time import sleep
import sys
 
COM_PORT = "COM5"
BAUD_RATES = 115200
ser = serial.Serial(COM_PORT, BAUD_RATES)
 
try:
    speed = 0
    inc = 0.1
    while True:
        #send command
        header = 0xFE
        cmd = 0x01
        argNum = 2
        forward = int(speed*255)
        turn = int(speed*255)
        msg = [header,cmd,argNum,forward,turn]
        print(msg)
        #compute checksum
        checksum = 0
        for ch in msg:
            checksum += ch
        checksum = checksum%256
        msg.append(checksum)
        ser.write(bytearray(msg))

        speed += inc
        if speed > 1:
            speed = 1
            inc = -inc
        elif speed < 0:
            speed = 0
            inc = -inc

        #receive status
        while ser.in_waiting:
            try:
                msg = ser.readline().decode()  # 接收回應訊息並解碼
                print('控制板狀態：', msg)
            except:
                print("invalid message")

        sleep(1)

except KeyboardInterrupt:
    ser.close()