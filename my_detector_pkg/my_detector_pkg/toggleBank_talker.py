import rclpy
#from my_detector_pkg.toggleBank import button
from rclpy.node import Node
import time
from std_msgs.msg import Bool


class Talker(Node):
    def __init__(self):
        super().__init__('Talker')
        self.publisher = self.create_publisher(Bool, 'disarmedEnabled', 10)

    def run(self):
        while rclpy.ok():
            disarmed = True

            self.get_logger().info("Disarmed: " and str(disarmed))

            msg = Bool()
            msg.data = disarmed
            self.publisher.publish(msg)

            time.sleep(0.25)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
