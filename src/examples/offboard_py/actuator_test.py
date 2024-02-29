#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from px4_msgs.msg import ActuatorServos

class ActuatorTestNode(Node):
    def __init__(self):
        super().__init__('actuator_test_node')
        # Adjust '/actuator_controls' to your specific topic for actuator commands
        self.publisher = self.create_publisher(ActuatorControls, '/actuator_controls', 10)
        
        # Delay to ensure subscribers are ready, then start testing actuators
        self.timer = self.create_timer(2, self.test_actuators)

    def test_actuators(self):
        # Test sequence for each actuator. Adjust as necessary for your application.
        # Here, we're assuming a simple back-and-forth for each actuator, one at a time.
        test_values = [-1.0, 1.0, float('nan')]  # Min, Max, and disarm for each actuator
        for i in range(ActuatorControls.NUM_CONTROLS):
            for value in test_values:
                msg = ActuatorControls()
                msg.timestamp = self.get_clock().now().to_msg().sec * 1_000_000  # Convert to microseconds
                msg.timestamp_sample = msg.timestamp  # Simplification for example
                
                # Set control value for current actuator, others to NaN
                msg.control = [float('nan')] * ActuatorControls.NUM_CONTROLS
                msg.control[i] = value
                
                self.publisher.publish(msg)
                self.get_logger().info(f'Publishing to actuator {i} with control value: {value}')
                rclpy.spin_once(self, timeout_sec=2)  # Wait a bit between commands

def main(args=None):
    rclpy.init(args=args)
    actuator_test_node = ActuatorTestNode()
    try:
        rclpy.spin(actuator_test_node)
    except KeyboardInterrupt:
        pass
    finally:
        actuator_test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
