import apriltag
import cv2
import yaml

tagSize = 30
with open("calibration.yml", "r") as f:
    calib = yaml.load(f)
    print(calib)
    camParam = [
        calib["camera_matrix"]["data"][0],
        calib["camera_matrix"]["data"][4],
        calib["camera_matrix"]["data"][2],
        calib["camera_matrix"]["data"][5],
    ]
    targetW = calib["image_width"]
    targetH = calib["image_height"]

cap = cv2.VideoCapture(2)
detector = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11") )

while(True):
    ret, frame = cap.read()
    frame = cv2.resize(frame, (targetW, targetH), interpolation=cv2.INTER_CUBIC)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    tags = detector.detect(gray)
    for tag in tags:
        cv2.line(frame, tuple(tag.corners[0].astype(int)), tuple(tag.corners[1].astype(int)), (0, 255, 0), 2)
        cv2.line(frame, tuple(tag.corners[1].astype(int)), tuple(tag.corners[2].astype(int)), (0, 0, 255), 2)
        cv2.line(frame, tuple(tag.corners[2].astype(int)), tuple(tag.corners[3].astype(int)), (255, 0, 0), 2)
        cv2.line(frame, tuple(tag.corners[3].astype(int)), tuple(tag.corners[0].astype(int)), (255, 0, 0), 2)

        pose, e0, e1 = detector.detection_pose(tag,camParam,tagSize)
        distZ = pose[2][3]
        cv2.putText(frame, "{:.2f}".format(distZ), tuple(tag.center.astype(int)), cv2.FONT_HERSHEY_SIMPLEX,0.5, (0, 0, 255), 1, cv2.LINE_AA)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()