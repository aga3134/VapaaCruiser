import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')
        self.pub = self.create_publisher(Twist,"car_cmd", 10)
        self.sub = self.create_subscription(Joy,"joy",self.ReceiveJoyCmd,10)
    
    def ReceiveJoyCmd(self,cmd):
        turn = cmd.axes[0]
        forward = cmd.axes[1]
        msg = Twist()
        msg.linear.x = forward
        msg.angular.z = turn
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    jc = JoyControl()
    rclpy.spin(jc)

    jc.destroy_node()
    rclpy.shutdown()

if __name__  ==  "__main__":
    main()
