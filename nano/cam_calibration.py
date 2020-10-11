import cv2
import numpy as np

gridX = 8
gridY = 6
gridSize = 18
targetW = 320
targetH = 240
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cap = cv2.VideoCapture(2)

objPtArr = []
imagePtArr = []
objPt = np.zeros((gridX*gridY,3), np.float32)
for x in range(gridX):
    for y in range(gridY):
        objPt[x*gridY+y] = [x*gridSize,y*gridSize,0]

imageNum = 0
while(True):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (targetW, targetH), interpolation=cv2.INTER_CUBIC)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, (gridX,gridY),None)
    
    if ret == True:
        subCorners = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        frame = cv2.drawChessboardCorners(frame, (gridX,gridY), subCorners,ret)

    cv2.imshow('frame', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"): #quit calibration
        break
    elif key == ord("a"): #add image for calibration
        if ret == True:
            objPtArr.append(objPt)
            imagePtArr.append(subCorners)
            imageNum += 1
            print("add image # %d" % imageNum)
        else:
            print("no chess board found")
    elif key == ord("c"): #do calibration
        ret, mat, distort, rotate, translate = cv2.calibrateCamera(objPtArr, imagePtArr, gray.shape[::-1],None,None)
        print(mat)
        print(distort)
        print(rotate)
        print(translate)


cap.release()
cv2.destroyAllWindows()