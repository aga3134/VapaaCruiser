import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage,Image, CameraInfo
from std_msgs.msg import Empty
import datetime
import os

class CamCapture(Node):
    def __init__(self):
        super().__init__("cam_capture")
        self.declare_parameter("camID")
        self.declare_parameter("width")
        self.declare_parameter("height")
        self.declare_parameter("rate")
        self.declare_parameter("flipH")
        self.declare_parameter("flipV")
        self.declare_parameter("savePath")

        self.camID = Parameter("camID", Parameter.Type.INTEGER, 0).value
        self.width = Parameter("width", Parameter.Type.INTEGER, 1280).value
        self.height = Parameter("height", Parameter.Type.INTEGER, 720).value
        self.rate = Parameter("rate", Parameter.Type.INTEGER, 60).value
        self.flipMethod = Parameter("flipMethod", Parameter.Type.INTEGER, 0).value
        self.savePath = Parameter("savePath", Parameter.Type.STRING, "saveImage/").value

        self.br = CvBridge()
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline(capture_width=self.width, capture_height=self.height, display_width=self.width, display_height=self.height, framerate=self.rate, flip_method=self.flipMethod), cv2.CAP_GSTREAMER)
        self.frame = None
        self.frameID = 1
        self.pub = self.create_publisher(Image, "cam_capture/image", 1)
        self.sub = self.create_subscription(Empty,"cam_capture/save",self.SaveImage,1)
        self.timer = self.create_timer(1.0/self.rate, self.ReadFromCam)
        
        self.get_logger().info("using camera id=%d, w=%d, h=%d, rate=%f" % (self.camID,self.width,self.height,self.rate))

    def gstreamer_pipeline(
        self,
        capture_width=1280,
        capture_height=720,
        display_width=1280,
        display_height=720,
        framerate=60,
        flip_method=0,
    ):
        return (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), "
            "width=(int)%d, height=(int)%d, "
            "format=(string)NV12, framerate=(fraction)%d/1 ! "
            "nvvidconv flip-method=%d ! "
            "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! appsink"
            % (
                capture_width,
                capture_height,
                framerate,
                flip_method,
                display_width,
                display_height,
            )
        )

    def SaveImage(self,msg):
        if self.frame is not None:
            if not os.path.exists(self.savePath):
                os.makedirs(self.savePath)

            now = datetime.datetime.now()
            filename = self.savePath+now.strftime("%Y-%m-%d_%H-%M-%S")+".jpg"
            self.get_logger().info("save image "+filename)
            cv2.imwrite(filename, self.frame)

    def ReadFromCam(self):
        ret, self.frame = self.cap.read()
        msg = self.br.cv2_to_imgmsg(self.frame,encoding='bgr8')
        self.pub.publish(msg)
        #cv2.imshow('frame',self.frame)
        #cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    cc = CamCapture()
    rclpy.spin(cc)
    cc.destroy_node()
    rclpy.shutdown()

if __name__  ==  "__main__":
    main()

