import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        self.pub = self.create_publisher(Twist,"car_cmd", 1)
        self.sub = self.create_subscription(Joy,"joy",self.ReceiveJoyCmd,1)
        self.timer = self.create_timer(0.03, self.PublishCommand)
        self.turn = 0.0
        self.forward = 0.0
    
    def ReceiveJoyCmd(self,cmd):
        self.turn = -cmd.axes[0]
        speed = math.sqrt(cmd.axes[0]*cmd.axes[0]+cmd.axes[1]*cmd.axes[1])
        #use speed with sign of axes[1]
        self.forward = math.copysign(speed, cmd.axes[1])
        
    def PublishCommand(self):
        msg = Twist()
        msg.linear.x = self.forward
        msg.angular.z = self.turn
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    jc = JoyControl()
    rclpy.spin(jc)

    jc.destroy_node()
    rclpy.shutdown()

if __name__  ==  "__main__":
    main()
