import rclpy
from my_detector_pkg.toggleBank import Button
from rclpy.node import Node
import time
from std_msgs.msg import Bool


class Talker(Node):


    def __init__(self):
        self.button = Button()
        super().__init__('Talker')
        self.publisher = self.create_publisher(Bool, 'disarmedEnabled', 10)
        self.disarmed = True

    def run(self):
        while rclpy.ok():
            # If it returns true it will switch disarmed state
            if(self.button.getButton()):
                self.disarmed = not self.disarmed

            self.get_logger().info(str(self.disarmed))

            msg = Bool()
            msg.data = self.disarmed
            self.publisher.publish(msg)

            time.sleep(1)

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
