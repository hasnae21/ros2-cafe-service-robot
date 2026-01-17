#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import sys

class TableNavigator(Node):
    def __init__(self):
        super().__init__('table_navigator')
        
        # Client d'action pour Nav2
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Charger les positions des tables
        self.table_positions = self.load_table_positions()
        
        self.get_logger().info('Table Navigator initialisé')
        self.get_logger().info(f'Tables disponibles: {list(self.table_positions.keys())}')
    
    def load_table_positions(self):
        """Charge les positions des tables depuis un fichier YAML"""
        try:
            # Chemin vers votre fichier de configuration
            config_file = '/home/ros2_ws_project/src/ros2-slam-auto-navigation/config/table_positions.yaml'
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                return config['tables']
        except Exception as e:
            self.get_logger().error(f'Erreur lors du chargement des positions: {e}')
            # Positions par défaut si le fichier n'existe pas
            return {
                'table_1': {
                    'position': {'x': 2.5, 'y': 3.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
                },
                'table_2': {
                    'position': {'x': -1.5, 'y': 2.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.707, 'w': 0.707}
                },
                'table_3': {
                    'position': {'x': 0.0, 'y': -2.5, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 1.0, 'w': 0.0}
                }
            }
    
    def navigate_to_table(self, table_name):
        """Envoie le robot vers une table spécifique"""
        if table_name not in self.table_positions:
            self.get_logger().error(f'Table "{table_name}" non trouvée!')
            self.get_logger().info(f'Tables disponibles: {list(self.table_positions.keys())}')
            return False
        
        # Attendre que le serveur d'action soit disponible
        self.get_logger().info('Attente du serveur de navigation...')
        self.nav_to_pose_client.wait_for_server()
        
        # Créer le goal de navigation
        table_pos = self.table_positions[table_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        # Position
        goal_msg.pose.pose.position.x = table_pos['position']['x']
        goal_msg.pose.pose.position.y = table_pos['position']['y']
        goal_msg.pose.pose.position.z = table_pos['position']['z']
        
        # Orientation
        goal_msg.pose.pose.orientation.x = table_pos['orientation']['x']
        goal_msg.pose.pose.orientation.y = table_pos['orientation']['y']
        goal_msg.pose.pose.orientation.z = table_pos['orientation']['z']
        goal_msg.pose.pose.orientation.w = table_pos['orientation']['w']
        
        self.get_logger().info(f'Navigation vers {table_name}...')
        self.get_logger().info(f'Position: x={table_pos["position"]["x"]}, y={table_pos["position"]["y"]}')
        
        # Envoyer le goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        
        return True
    
    def feedback_callback(self, feedback_msg):
        """Callback pour recevoir les feedbacks pendant la navigation"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Distance restante: {feedback.distance_remaining:.2f}m',
            throttle_duration_sec=2.0  # Log toutes les 2 secondes
        )
    
    def goal_response_callback(self, future):
        """Callback quand le goal est accepté ou rejeté"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejeté par le serveur de navigation!')
            return
        
        self.get_logger().info('Goal accepté! Navigation en cours...')
        
        # Attendre le résultat
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """Callback quand la navigation est terminée"""
        result = future.result().result
        self.get_logger().info('Navigation terminée!')

def main(args=None):
    rclpy.init(args=args)
    
    navigator = TableNavigator()
    
    # Vérifier si un nom de table est fourni en argument
    if len(sys.argv) < 2:
        navigator.get_logger().info('Usage: ros2 run ros2_slam_auto_navigation navigate_to_table.py <table_name>')
        navigator.get_logger().info(f'Tables disponibles: {list(navigator.table_positions.keys())}')
        navigator.destroy_node()
        rclpy.shutdown()
        return
    
    table_name = sys.argv[1]
    
    # Lancer la navigation
    if navigator.navigate_to_table(table_name):
        # Garder le nœud actif pendant la navigation
        try:
            rclpy.spin(navigator)
        except KeyboardInterrupt:
            navigator.get_logger().info('Navigation interrompue par l\'utilisateur')
    
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()