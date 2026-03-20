import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

# create node ros2 personalized
class WaypointServer(Node):
    #initialized node
    def __init__(self):
        super().__init__('wp_server')
        
        # definition of the Waypoints (Matrix 11x3)
        self.waypoints = np.array([
            [0, 0, 0],
            [5, 0, math.pi/2],
            [5, 5, 5/4*math.pi],
            [-5, -5, math.pi/2],
            [-5, 5, 0],
            [0, 0, 0],
            [3, 3, 3/4*math.pi],
            [-3, 0, 3/2*math.pi],
            [0, -3, math.pi/4],
            [3, 0, math.pi/2],
            [0, 0, 3/2*math.pi]
        ])

       # subscriber /wp_request
        self.subscription = self.create_subscription(
            Int32,
            '/wp_request',
            self.listener_callback,
            10)
        
        # Publisher per /next_waypoint
        self.publisher = self.create_publisher(PoseStamped, '/next_waypoint', 10)
        
        self.get_logger().info('Waypoint Server avviato e in attesa di richieste...')

    def listener_callback(self, msg):
        k = msg.data - 1 
        
        if 0 <= k < len(self.waypoints):
            x = self.waypoints[k][0]
            y = self.waypoints[k][1]
            th = self.waypoints[k][2]

            qw = math.cos(th / 2.0)
            qz = math.sin(th / 2.0)

            wp_msg = PoseStamped()
            wp_msg.header.frame_id = 'map'  
            wp_msg.header.stamp = self.get_clock().now().to_msg()  
            
            wp_msg.pose.position.x = float(x)
            wp_msg.pose.position.y = float(y)
            wp_msg.pose.position.z = 0.0
            
            wp_msg.pose.orientation.x = 0.0
            wp_msg.pose.orientation.y = 0.0
            wp_msg.pose.orientation.z = float(qz)
            wp_msg.pose.orientation.w = float(qw)

            self.publisher.publish(wp_msg)
            
            self.get_logger().info(f'Waypoint {msg.data}: [{x:.2f}, {y:.2f}, {th:.2f}]')
        else:
            self.get_logger().warn(f'Indice {msg.data} fuori range!')


def main(args=None):
    rclpy.init(args=args)
    waypoint_server = WaypointServer()
    
    try:
        rclpy.spin(waypoint_server)
    except KeyboardInterrupt:
        pass
    finally:
        waypoint_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
