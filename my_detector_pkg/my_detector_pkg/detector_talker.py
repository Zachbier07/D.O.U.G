import rclpy
from rclpy.node import Node
import time
from my_detector_pkg.handTracking import HandTracker
from std_msgs.msg import Bool


class JazzyTalker(Node):
    def __init__(self):
        super().__init__('jazzy_talker')
        self.get_logger().info("Jazzy Talker node started!")
        self.tracker = HandTracker()  # make it an attribute
        self.publisher = self.create_publisher(Bool, 'handDetection', 10)

    def run(self):
        while rclpy.ok():
            detected = self.tracker.process_frame()

            self.get_logger().info(str(detected))

            msg = Bool()
            msg.data = detected  
            self.publisher.publish(msg)

            time.sleep(0.025)

def main(args=None):
    rclpy.init(args=args)
    node = JazzyTalker()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.tracker.release()  # release camera if needed
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
