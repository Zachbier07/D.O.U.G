import rclpy
from my_detector_pkg.toggleBank import Button
from rclpy.node import Node
import time
from std_msgs.msg import Bool
import Jetson.GPIO as GPIO


import Jetson.GPIO as GPIO
import time

class Button:
    def __init__(self):
        # Set up the GPIO pin you wired your button to (physical pin 18 = BCM 24)
        self.pin = 13
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        # Internal state to avoid multiple triggers from one press
        self.last_state = GPIO.input(self.pin)
        self.last_time = time.time()

    def getButton(self, debounce_time=0.2):
        current_state = GPIO.input(self.pin)

        # Detect press 
        if self.last_state == GPIO.HIGH and current_state == GPIO.LOW:
            now = time.time()
            if now - self.last_time > debounce_time:
                self.last_time = now
                self.last_state = current_state
                return True

        # Update state
        self.last_state = current_state
        return False

    def cleanup(self):
        GPIO.cleanup(self.pin)



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
