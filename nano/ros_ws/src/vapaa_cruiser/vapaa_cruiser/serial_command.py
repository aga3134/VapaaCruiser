import rclpy
from rclpy.node import Node
import serial
import threading
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter
import time

class SerialCommand(Node):
    def __init__(self):
        super().__init__('serial_command')
        self.declare_parameter("port")
        self.declare_parameter("baud",115200)
        self.port = Parameter("port", Parameter.Type.STRING, "/dev/ttyUSB0").value
        self.baud = Parameter("baud", Parameter.Type.INTEGER, 115200).value

        self.get_logger().info("using port: %s, baud rate: %d" % (self.port,self.baud))
        
        self.state = "FORWARD"
        self.stateTime = 0
        self.ser = serial.Serial(self.port, self.baud)
        self.pub = self.create_publisher(String, "car_state", 1)
        self.sub = self.create_subscription(Twist,'car_cmd',self.ReceiveCmd,1)

    def ReceiveCmd(self,cmd):
        curForward = cmd.linear.x
        curTurn = cmd.angular.z

        if curForward > 1:
            curForward = 1
        if curForward < -1:
            curForward = -1
        if curTurn > 1:
            curTurn = 1
        if curTurn < -1:
            curTurn = -1
        
        #simulate double click
        if self.state == "FORWARD":
            if curForward < 0:
                self.state = "BACKWARD_CLICK1"
                self.stateTime = time.time()
        elif self.state == "BACKWARD_CLICK1":
            if curForward >= 0:
                self.state = "FORWARD"
            else:
                t = time.time()
                if t - self.stateTime > 0.7:
                    #self.get_logger().info("stateTime: %f" % (t-self.stateTime))
                    self.state = "BACKWARD_PAUSE"
                    self.stateTime = t
        elif self.state == "BACKWARD_PAUSE":
            if curForward >= 0:
                self.state = "FORWARD"
            else:
                t = time.time()
                if t - self.stateTime > 0.4:
                    self.state = "BACKWOARD_CLICK2"
                    #讓車子從低速開始倒退，避免瞬間加速過大
                    curForward = -0.01
                    
        elif self.state == "BACKWOARD_CLICK2":
            if curForward > 0: #車有前進才換到forward，不然留在此state
                self.state = "FORWARD"
            elif curForward < -0.05: #要做到double click的後退訊號會讓車子倒衝太快，這邊把最高速度降低
                curForward = -0.05

        forward = curForward
        if self.state == "BACKWARD_PAUSE":
            forward = 0

        #self.get_logger().info("forward: %f, turn: %f" % (forward,curTurn))
        #send command
        header = 0xFE
        cmd = 0x01
        argNum = 2
        forward = int((forward+1)*0.5*255)
        turn = int((curTurn+1)*0.5*255)
        msg = [header,cmd,argNum,forward,turn]
        
        #compute checksum
        checksum = 0
        for ch in msg:
            checksum += ch
        checksum = checksum%256
        msg.append(checksum)
        #self.get_logger().info("%x %x %x %x %x %x" % (msg[0],msg[1],msg[2],msg[3],msg[4],msg[5]))
        self.ser.write(bytearray(msg))


    def ReceiveState(self,state):
        msg = String()
        msg.data = state
        self.pub.publish(msg)

    def ReadFromSerial(self):
       while rclpy.ok():
            while self.ser.in_waiting:
                try:
                    msg = self.ser.readline().decode()  # 接收回應訊息並解碼
                    self.ReceiveState(msg)
                except:
                    self.get_logger().info("serial invalid message")

def main(args=None):
    rclpy.init(args=args)
    sc = SerialCommand()
    #use a separate thread to read data from serial
    thread = threading.Thread(target=sc.ReadFromSerial)
    thread.start()
    
    rclpy.spin(sc)

    sc.destroy_node()
    rclpy.shutdown()

if __name__  ==  "__main__":
    main()
