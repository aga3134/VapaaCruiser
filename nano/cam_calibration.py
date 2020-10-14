import cv2
import numpy as np
import yaml

gridX = 8
gridY = 6
gridSize = 25
targetW = 640
targetH = 480
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

cap = cv2.VideoCapture(2)

imageNum = 0
objPtArr = []
imagePtArr = []
objPt = np.zeros((gridX*gridY,3), np.float32)
for y in range(gridY):
    for x in range(gridX):
        objPt[y*gridX+x] = [x*gridSize,y*gridSize,0]

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
        if imageNum < 10:
            print("please add at least 10 chessboard images with different positions & angles")
        else:
            ret, mat, distort, rotate, translate = cv2.calibrateCamera(objPtArr, imagePtArr, gray.shape[::-1],None,None)
            output = {
                "image_width": targetW,
                "image_height": targetH,
                "camera_matrix": {
                    "rows": mat.shape[0],
                    "cols": mat.shape[1],
                    "data": mat.flatten().tolist()
                },
                "distortion_coefficients": {
                    "rows": distort.shape[0],
                    "cols": distort.shape[1],
                    "data": distort.flatten().tolist()
                },
            }
            filename = "calibration.yml"
            with open(filename, "w") as f:
                yaml.dump(output, f)
                print(output)
                print("save calibration result to %s" % filename)

            #reset 
            imageNum = 0
            objPtArr = []
            imagePtArr = []

cap.release()
cv2.destroyAllWindows()