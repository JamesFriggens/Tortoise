import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class SerialSender(Node):
    def __init__(self):
        super().__init__('motor_serial_node')

        # Open serial port
        try:
            self.port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
            self.get_logger().info('Serial port opened successfully.')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.port = None

        # Subscribe to /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

    def cmd_vel_callback(self, msg: Twist):
        # Check for forward movement
        if msg.linear.x > 0.0:
            self.send_serial(b'1')
        elif msg.linear.x < 0.0:
            self.send_serial(b'0')  # Optional: backward
        elif msg.angular.z != 0.0:
            self.send_serial(b'3')  # Optional: turn
        else:
            self.send_serial(b'0')  # Stop or idle

    def send_serial(self, value: bytes):
        if self.port and self.port.is_open:
            try:
                self.port.write(value)
                self.get_logger().info(f'Sent: {value}')
            except Exception as e:
                self.get_logger().error(f'Failed to send: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
