#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from f110_msgs.msg import WpntArray
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from steering_lookup.lookup_steer_angle import LookupSteerAngle


class Controller(Node):
    def __init__(self):
        super().__init__('map_controller')

        # Get parameters for the MAP controller
        self.declare_parameters(
            namespace='',
            parameters=[
                ('q_map', 0.3),
                ('m_map', 0.2),
                ('t_clip_min', 0.5),
                ('t_clip_max', 3.0),
                ('LU_table', 'NUC1_pacejka')
            ]
        )
        
        self.param_q_map = self.get_parameter('q_map').value
        self.param_m_map = self.get_parameter('m_map').value
        self.param_t_clip_min = self.get_parameter('t_clip_min').value
        self.param_t_clip_max = self.get_parameter('t_clip_max').value
        LUT_name = self.get_parameter('LU_table').value

        # Load lookup table to calculate steering angle
        self.steer_lookup = LookupSteerAngle(LUT_name)

        # Set loop rate in hertz
        self.loop_rate = 40

        # Initialize variables
        self.position = None  # current position in map frame [x, y, theta]
        self.waypoints = None  # waypoints in map frame [x, y, speed]

        # Publisher to publish lookahead point and steering angle
        self.lookahead_pub = self.create_publisher(
            Marker, 'lookahead_point', 10)

        # Publisher for steering and speed command
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 
            '/vesc/high_level/ackermann_cmd_mux/input/nav_1', 
            10)

        # Subscribers to get waypoints and the position of the car
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/car_state/pose', 
            self.car_state_cb, 
            10)
        
        self.waypoint_sub = self.create_subscription(
            WpntArray, 
            '/global_waypoints', 
            self.waypoints_cb, 
            10)
        
        # Timer for control loop
        self.timer = self.create_timer(1.0/self.loop_rate, self.control_loop)
        
        self.get_logger().info('MAP Controller initialized')

    def car_state_cb(self, data):
        """
        The callback function for the '/car_state/pose' subscriber.
        
        Args:
            data (PoseStamped): The message containing the current pose of the car.
        """
        x = data.pose.position.x
        y = data.pose.position.y
        theta = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y,
                                      data.pose.orientation.z, data.pose.orientation.w])[2]
        self.position = [x, y, theta]

    def waypoints_cb(self, data):
        """
        The callback function for the '/global_waypoints' subscriber.
        
        Args:
            data (WpntArray): The message containing the global waypoints.
        """
        self.waypoints = np.empty((len(data.wpnts), 3), dtype=np.float32)
        for i, waypoint in enumerate(data.wpnts):
            speed = waypoint.vx_mps
            self.waypoints[i] = [waypoint.x_m, waypoint.y_m, speed]

    def control_loop(self):
        """
        Control loop for the MAP controller.
        """
        # Wait for position and waypoints
        if self.position is None or self.waypoints is None:
            return

        # Send speed and steering commands to mux
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = self.get_clock().now().to_msg()
        ack_msg.header.frame_id = 'base_link'

        if self.waypoints.shape[0] > 2:
            idx_nearest_waypoint = self.nearest_waypoint(self.position[:2], self.waypoints[:, :2])

            # Desired speed at waypoint closest to car
            target_speed = self.waypoints[idx_nearest_waypoint, 2]

            # Calculate lookahead_distance
            # Define lookahead distance as an affine function with tuning parameter m and q
            lookahead_distance = self.param_q_map + target_speed*self.param_m_map
            lookahead_distance = np.clip(lookahead_distance, self.param_t_clip_min, self.param_t_clip_max)
            lookahead_point = self.waypoint_at_distance_infront_car(
                lookahead_distance,
                self.waypoints[:, :2],
                idx_nearest_waypoint)

            if lookahead_point.any() is not None:
                # Vector from the current position to the point at lookahead distance
                position_la_vector = np.array([lookahead_point[0] - self.position[0], 
                                              lookahead_point[1] - self.position[1]])
                yaw = self.position[2]
                eta = np.arcsin(np.dot([-np.sin(yaw), np.cos(yaw)], position_la_vector)/np.linalg.norm(position_la_vector))
                lat_acc = 2*target_speed**2 / lookahead_distance * np.sin(eta)

                steering_angle = self.steer_lookup.lookup_steer_angle(lat_acc, target_speed)

                ack_msg.drive.steering_angle = steering_angle
                ack_msg.drive.speed = max(target_speed, 0)  # no negative speed

                self.visualize_lookahead(lookahead_point)
                self.visualize_steering(steering_angle)

        # If there are no waypoints, publish zero speed and steer to STOP
        else:
            ack_msg.drive.speed = 0
            ack_msg.drive.steering_angle = 0
            self.get_logger().error('[MAP Controller]: Received no waypoints. STOPPING!!!')

        # Always publish ackermann msg
        self.drive_pub.publish(ack_msg)

    @staticmethod
    def distance(point1, point2):
        """
        Calculate the Euclidean distance between two points.
        """
        return np.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    @staticmethod
    def nearest_waypoint(position, waypoints):
        """
        Find the index of the closest waypoint to the current position.
        """
        return np.argmin(np.sum((waypoints - position)**2, axis=1))

    def waypoint_at_distance_infront_car(self, distance, waypoints, idx_waypoint_behind_car):
        """
        Find waypoint at distance in front of the car.
        """
        num_waypoints = waypoints.shape[0]
        lookahead_point = None
        distance_traveled = 0.0
        idx_curr = idx_waypoint_behind_car
        
        for i in range(num_waypoints):
            idx_next = (idx_curr + 1) % num_waypoints
            distance_segment = self.distance(waypoints[idx_curr], waypoints[idx_next])
            
            if distance_traveled + distance_segment > distance:
                # Interpolate waypoint at exactly lookahead_distance
                t = (distance - distance_traveled) / distance_segment
                lookahead_point = waypoints[idx_curr] + t * (waypoints[idx_next] - waypoints[idx_curr])
                break
            
            distance_traveled += distance_segment
            idx_curr = idx_next
            
            # Avoid infinite loop by stopping after going through all waypoints
            if idx_curr == idx_waypoint_behind_car:
                lookahead_point = waypoints[idx_curr]
                break
        
        return lookahead_point if lookahead_point is not None else np.array([0, 0])

    def visualize_steering(self, theta):
        """
        Publish a marker to visualize the steering angle.
        """
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "base_link"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.ns = "steering_angle"
        lookahead_marker.id = 0
        lookahead_marker.type = Marker.ARROW
        lookahead_marker.action = Marker.ADD
        
        length = 0.5
        lookahead_marker.scale.x = length
        lookahead_marker.scale.y = 0.1
        lookahead_marker.scale.z = 0.1
        
        lookahead_marker.color.a = 1.0
        lookahead_marker.color.r = 1.0
        lookahead_marker.color.g = 0.0
        lookahead_marker.color.b = 0.0
        
        lookahead_marker.pose.position.x = 0.25
        lookahead_marker.pose.position.y = 0
        lookahead_marker.pose.position.z = 0
        
        quaternions = quaternion_from_euler(0, 0, theta)
        lookahead_marker.pose.orientation.x = quaternions[0]
        lookahead_marker.pose.orientation.y = quaternions[1]
        lookahead_marker.pose.orientation.z = quaternions[2]
        lookahead_marker.pose.orientation.w = quaternions[3]
        
        self.lookahead_pub.publish(lookahead_marker)

    def visualize_lookahead(self, lookahead_point):
        """
        Publish a marker to visualize the lookahead point.
        """
        lookahead_marker = Marker()
        lookahead_marker.header.frame_id = "map"
        lookahead_marker.header.stamp = self.get_clock().now().to_msg()
        lookahead_marker.ns = "lookahead_point"
        lookahead_marker.id = 1
        lookahead_marker.type = Marker.SPHERE
        lookahead_marker.action = Marker.ADD
        
        lookahead_marker.scale.x = 0.2
        lookahead_marker.scale.y = 0.2
        lookahead_marker.scale.z = 0.2
        
        lookahead_marker.color.a = 1.0
        lookahead_marker.color.r = 0.0
        lookahead_marker.color.g = 1.0
        lookahead_marker.color.b = 0.0
        
        lookahead_marker.pose.position.x = lookahead_point[0]
        lookahead_marker.pose.position.y = lookahead_point[1]
        lookahead_marker.pose.position.z = 0
        
        lookahead_marker.pose.orientation.x = 0
        lookahead_marker.pose.orientation.y = 0
        lookahead_marker.pose.orientation.z = 0
        lookahead_marker.pose.orientation.w = 1
        
        self.lookahead_pub.publish(lookahead_marker)


def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()