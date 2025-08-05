import rclpy
from rclpy.node import Node
import serial

class SerialSender(Node):
    def __init__(self):
        super().__init__('motor_serial_node')
        self.port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        self.timer = self.create_timer(1.0, self.send_message)  # Send every 1 second

    def send_message(self):
        try:
            self.port.write(b'1')  # Send byte "1"
            self.get_logger().info('Sent: 1')
        except Exception as e:
            self.get_logger().error(f'Failed to send: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialSender()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
