import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

# crea nodo ros2 personalizzato
class WaypointServer(Node):
    #inizializzazione nodo
    def __init__(self):
        super().__init__('wp_server')
        
        # Definizione dei Waypoints (Matrice 11x3)
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

        # Sottoscrizione al topic /wp_request:
        # ascolta /wp_request
        # messaggio int32
        # quando arriva chiama listener_callback
       
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
        # Il messaggio ricevuto k è l'indice (nota: Python usa base 0)
        # Se il tuo script MATLAB invia indici partendo da 1, sottrai 1 qui:
        k = msg.data - 1 
        
        # Controllo che l'indice sia nel range
        if 0 <= k < len(self.waypoints):
            # estrazione waypoint
            x = self.waypoints[k][0]
            y = self.waypoints[k][1]
            th = self.waypoints[k][2]

            # Calcolo quaternioni (solo Z e W per rotazione sul piano)
            qw = math.cos(th / 2.0)
            qz = math.sin(th / 2.0)

            # Preparazione del messaggio PoseStamped
            wp_msg = PoseStamped()
            wp_msg.header.frame_id = 'map'  # coordinate espresse nel frame map
            wp_msg.header.stamp = self.get_clock().now().to_msg()  #sincronizzazione, TF, validità messaggio
            
            wp_msg.pose.position.x = float(x)
            wp_msg.pose.position.y = float(y)
            wp_msg.pose.position.z = 0.0
            
            wp_msg.pose.orientation.x = 0.0
            wp_msg.pose.orientation.y = 0.0
            wp_msg.pose.orientation.z = float(qz)
            wp_msg.pose.orientation.w = float(qw)

            # Invio del messaggio a MATLAB
            self.publisher.publish(wp_msg)
            
            # per vedere cosa succede nel terminale ed eventualmente debuggare
            self.get_logger().info(f'Waypoint {msg.data}: [{x:.2f}, {y:.2f}, {th:.2f}]')
        else:
            self.get_logger().warn(f'Indice {msg.data} fuori range!')


# definisce funzione principale di python
# args=None → permette a ROS2 di passare argomenti da terminale
def main(args=None):
    # inizializza ros2 in python, senza questo non parte ros e nodo crasha
    rclpy.init(args=args)
    # creazione nodo (eseguito __init__()) 
    waypoint_server = WaypointServer()
    
    # accende e tiene vivi nodi: /wp_request viene ascoltato and listener_callback() viene eseguita
    try:
        rclpy.spin(waypoint_server)
    # Serve a chiudere il nodo con Ctrl+C
    except KeyboardInterrupt:
        pass
    finally:
        # distrugge nodo, chiude publisher and subscriber
        waypoint_server.destroy_node()
        # spegne ros
        rclpy.shutdown()

# Esegui main() solo se questo file è lanciato direttamente
if __name__ == '__main__':
    main()
