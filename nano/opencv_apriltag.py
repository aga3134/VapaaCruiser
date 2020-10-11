import apriltag
import cv2

cap = cv2.VideoCapture(2)
detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11") )

while(True):
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    for tag in tags:
        cv2.line(frame, tuple(tag.corners[0].astype(int)), tuple(tag.corners[1].astype(int)), (0, 255, 0), 2)
        cv2.line(frame, tuple(tag.corners[1].astype(int)), tuple(tag.corners[2].astype(int)), (0, 0, 255), 2)
        cv2.line(frame, tuple(tag.corners[2].astype(int)), tuple(tag.corners[3].astype(int)), (255, 0, 0), 2)
        cv2.line(frame, tuple(tag.corners[3].astype(int)), tuple(tag.corners[0].astype(int)), (255, 0, 0), 2)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()