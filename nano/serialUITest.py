import pygame
from pygame.locals import *
import serial
from time import sleep
import threading

COM_PORT = "COM5"
BAUD_RATES = 115200
ser = serial.Serial(COM_PORT, BAUD_RATES)
done = False

#init pygame window
pygame.init()
screen = pygame.display.set_mode( (640,480) )
pygame.display.set_caption("Serial UI")
screen.fill((159, 182, 205))
font = pygame.font.Font(None, 17)


def display(str):
    text = font.render(str, True, (255, 255, 255), (159, 182, 205))
    textRect = text.get_rect()
    textRect.centerx = screen.get_rect().centerx
    textRect.centery = screen.get_rect().centery
    screen.blit(text, textRect)
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

        #send command
        header = 0xFE
        cmd = 0x01
        argNum = 2
        forward = int((curForward+1)*0.5*255)
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