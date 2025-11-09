import rclpy
from rclpy.node import Node
import time
from my_detector_pkg.handTracking import HandTracker
from std_msgs.msg import Bool


class Talker(Node):

    def __init__(self):
        super().__init__('Talker')
        self.tracker = HandTracker() 
        self.publisher = self.create_publisher(Bool, 'handDetection', 10)
        self.sub = self.create_subscription(
            Bool,
            'disarmedEnabled',
            self.toggle_callback,
            10
        )
        self.disarmed = False
    
    def toggle_callback(self, msg):
        self.disarmed = msg.data

    def run(self):
        while rclpy.ok():
            # Allow callbacks (subscriber) to run
            rclpy.spin_once(self, timeout_sec=0)

            if self.disarmed == False:
                detected = self.tracker.process_frame()
            else:
                detected = False
                self.get_logger().info("Disarmed")

            self.get_logger().info(str(detected))

            msg = Bool()
            msg.data = detected  
            self.publisher.publish(msg)

            time.sleep(0.025)

def main(args=None):
    rclpy.init(args=args)
    node = Talker()
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
