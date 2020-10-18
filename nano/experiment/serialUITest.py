import pygame
from pygame.locals import *
import serial
from time import sleep
import time
import threading

COM_PORT = "COM12"
BAUD_RATES = 115200
ser = serial.Serial(COM_PORT, BAUD_RATES)
done = False

#init pygame window
pygame.init()
screen = pygame.display.set_mode( (640,480) )
pygame.display.set_caption("Serial UI")
screen.fill((159, 182, 205))
font = pygame.font.Font(None, 20)


def display(str):
    cx = 120
    cy = 120
    arr = str.split(",")
    #lat lng
    text = font.render(arr[0]+","+arr[1], True, (255, 255, 255), (159, 182, 205))
    screen.blit(text, (cx,cy))
    #LF
    text = font.render(arr[2], True, (255, 255, 255), (159, 182, 205))
    screen.blit(text, (10,10))
    #F
    text = font.render(arr[3], True, (255, 255, 255), (159, 182, 205))
    screen.blit(text, (cx,10))
    #RF
    text = font.render(arr[4], True, (255, 255, 255), (159, 182, 205))
    screen.blit(text, (cx*2,10))
    #LB
    text = font.render(arr[5], True, (255, 255, 255), (159, 182, 205))
    screen.blit(text, (10,cy*2))
    #B
    text = font.render(arr[6], True, (255, 255, 255), (159, 182, 205))
    screen.blit(text, (cx,cy*2))
    #RB
    text = font.render(arr[7], True, (255, 255, 255), (159, 182, 205))
    screen.blit(text, (cx*2,cy*2))

    pygame.display.update()

def ReadFromSerial(ser):
    while not done:
        while ser.in_waiting:
            try:
                msg = ser.readline().decode()  # 接收回應訊息並解碼
                display(msg)
            except:
                print("invalid message")

if __name__ == "__main__":
    #use a separate thread to read data from serial & show in pygame window
    thread = threading.Thread(target=ReadFromSerial, args=(ser,))
    thread.start()

    #send command according to keyboard input
    targetTurn = 0
    targetForward = 0
    curTurn = 0
    curForward = 0
    rate = 0.5
    #simulate double click 
    state = "FORWARD"
    stateTime = 0
    while not done:
        pygame.event.pump()
        keys = pygame.key.get_pressed()

        targetTurn = 0
        targetForward = 0
        if keys[K_ESCAPE]:
            done = True
        if keys[K_UP]:
            targetForward = 1
            curForward = curForward*(1-rate)+targetForward*rate
        elif keys[K_DOWN]:
            targetForward = -1
            curForward = curForward*(1-rate)+targetForward*rate
        else:
            curForward = 0

        if keys[K_LEFT]:
            targetTurn = -1
            curTurn = curTurn*(1-rate)+targetTurn*rate
        elif keys[K_RIGHT]:
            targetTurn = 1
            curTurn = curTurn*(1-rate)+targetTurn*rate
        else:
            curTurn = 0

        if curForward > 1:
            curForward = 1
        if curForward < -1:
            curForward = -1
        if curTurn > 1:
            curTurn = 1
        if curTurn < -1:
            curTurn = -1

        #simulate double click
        if state == "FORWARD":
            if curForward < 0:
                state = "BACKWARD_CLICK1"
                stateTime = time.time()
        elif state == "BACKWARD_CLICK1":
            if curForward >= 0:
                state = "FORWARD"
            else:
                t = time.time()
                if t - stateTime > 0.7:
                    print(t-stateTime)
                    state = "BACKWARD_PAUSE"
                    stateTime = t
        elif state == "BACKWARD_PAUSE":
            if curForward >= 0:
                state = "FORWARD"
            else:
                t = time.time()
                if t - stateTime > 0.4:
                    state = "BACKWOARD_CLICK2"
                    #讓車子從低速開始倒退，避免瞬間加速過大
                    curForward = -0.1
                    
        elif state == "BACKWOARD_CLICK2":
            if curForward > 0: #車有前進才換到forward，不然留在此state
                state = "FORWARD"
            elif curForward < -0.8: #要做到double click的後退訊號會讓車子倒衝太快，這邊把最高速度降低
                curForward = -0.8

        forward = curForward
        if state == "BACKWARD_PAUSE":
            forward = 0

        #send command
        header = 0xFE
        cmd = 0x01
        argNum = 2
        forward = int((forward+1)*0.5*255)
        turn = int((curTurn+1)*0.5*255)
        msg = [header,cmd,argNum,forward,turn]
        print(msg)
        #compute checksum
        checksum = 0
        for ch in msg:
            checksum += ch
        checksum = checksum%256
        msg.append(checksum)
        ser.write(bytearray(msg))

        sleep(0.1)

    thread.join()