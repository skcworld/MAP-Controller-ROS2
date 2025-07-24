#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class TimeErrorTracker(Node):
    def __init__(self):
        super().__init__('time_error_tracker')
        
        # Subscribe to car pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/car_state/pose',
            self.pose_callback,
            10
        )
        
        self.get_logger().info('Time Error Tracker initialized')
    
    def pose_callback(self, msg):
        # Simple placeholder for time error tracking
        pass


def main(args=None):
    rclpy.init(args=args)
    tracker = TimeErrorTracker()
    
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()