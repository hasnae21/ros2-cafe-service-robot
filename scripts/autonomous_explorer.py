#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import numpy as np
import math
from scipy.ndimage import binary_dilation, label
import time

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Paramètres
        self.declare_parameter('explore_duration', 300)  # 5 minutes
        self.declare_parameter('frontier_threshold', 10)
        self.declare_parameter('safety_distance', 0.5)
        
        self.explore_duration = self.get_parameter('explore_duration').value
        self.frontier_threshold = self.get_parameter('frontier_threshold').value
        self.safety_distance = self.get_parameter('safety_distance').value
        
        # État
        self.map_data = None
        self.robot_pose = None
        self.exploring = True
        self.start_time = time.time()
        self.visited_frontiers = []
        self.exploration_complete = False
        self.goal_handle = None
        
        # Publishers et Subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Action client pour Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Timer pour l'exploration
        self.explore_timer = self.create_timer(3.0, self.explore_callback)
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🚀 EXPLORATEUR AUTONOME DÉMARRÉ')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'⏱️  Durée d\'exploration: {self.explore_duration}s')
        self.get_logger().info(f'📏 Seuil de frontière: {self.frontier_threshold} cellules')
        self.get_logger().info(f'🛡️  Distance de sécurité: {self.safety_distance}m')
        self.get_logger().info('=' * 60)
        
    def map_callback(self, msg):
        """Reçoit et stocke la carte"""
        self.map_data = msg
        
    def odom_callback(self, msg):
        """Reçoit la position du robot"""
        self.robot_pose = msg.pose.pose
        
    def explore_callback(self):
        """Boucle principale d'exploration"""
        # Vérifier le temps écoulé
        elapsed_time = time.time() - self.start_time
        remaining_time = self.explore_duration - elapsed_time
        
        if elapsed_time > self.explore_duration:
            if not self.exploration_complete:
                self.get_logger().info('=' * 60)
                self.get_logger().info('✅ EXPLORATION TERMINÉE PAR TIMEOUT')
                self.stop_exploration()
            return
        
        # Afficher le temps restant toutes les 30 secondes
        if int(elapsed_time) % 30 == 0:
            self.get_logger().info(f'⏳ Temps restant: {int(remaining_time)}s')
        
        # Vérifier si nous avons les données nécessaires
        if self.map_data is None:
            self.get_logger().warn('⏳ En attente de la carte...')
            return
            
        if self.robot_pose is None:
            self.get_logger().warn('⏳ En attente de l\'odométrie...')
            return
        
        if not self.exploring:
            return
        
        # Trouver et naviguer vers la prochaine frontière
        frontier = self.find_best_frontier()
        
        if frontier is not None:
            x, y = frontier
            self.get_logger().info(f'🎯 Nouvelle frontière: x={x:.2f}m, y={y:.2f}m')
            self.navigate_to_point(frontier)
        else:
            self.get_logger().info('🔍 Aucune frontière significative trouvée')
            self.get_logger().info('🔄 Rotation 360° pour scanner...')
            self.rotate_in_place()
            
            # Vérifier à nouveau après rotation
            frontier = self.find_best_frontier()
            if frontier is None:
                self.get_logger().info('✅ EXPLORATION COMPLÈTE - Aucune zone inexplorée')
                self.stop_exploration()
    
    def find_best_frontier(self):
        """Trouve la meilleure frontière à explorer"""
        if self.map_data is None or self.robot_pose is None:
            return None
        
        try:
            # Convertir les données de la carte
            width = self.map_data.info.width
            height = self.map_data.info.height
            resolution = self.map_data.info.resolution
            origin = self.map_data.info.origin
            
            # Créer un tableau numpy de la carte
            map_array = np.array(self.map_data.data).reshape((height, width))
            
            # Classification: -1=inconnu, 0=libre, 100=occupé
            unknown = (map_array == -1)
            free = (map_array == 0)
            occupied = (map_array > 50)
            
            # Vérifier qu'il y a des zones inconnues
            if not np.any(unknown):
                self.get_logger().info('📊 Carte complète - Aucune zone inconnue')
                return None
            
            # Dilater les zones libres pour trouver les frontières
            free_dilated = binary_dilation(free, iterations=2)
            
            # Frontières = zones dilatées qui touchent l'inconnu
            frontiers = free_dilated & unknown
            
            # Trouver les groupes de frontières
            labeled_frontiers, num_features = label(frontiers)
            
            if num_features == 0:
                return None
            
            # Position actuelle du robot en coordonnées de grille
            robot_x_grid = (self.robot_pose.position.x - origin.position.x) / resolution
            robot_y_grid = (self.robot_pose.position.y - origin.position.y) / resolution
            
            best_frontier = None
            best_score = -float('inf')
            
            self.get_logger().info(f'🔎 {num_features} groupes de frontières détectés')
            
            # Évaluer chaque groupe de frontières
            for i in range(1, num_features + 1):
                frontier_cells = np.argwhere(labeled_frontiers == i)
                frontier_size = len(frontier_cells)
                
                # Ignorer les petites frontières
                if frontier_size < self.frontier_threshold:
                    continue
                
                # Centroïde de la frontière (row, col)
                centroid_row = frontier_cells[:, 0].mean()
                centroid_col = frontier_cells[:, 1].mean()
                
                # Convertir en coordonnées du monde
                world_x = centroid_col * resolution + origin.position.x
                world_y = centroid_row * resolution + origin.position.y
                
                # Vérifier si déjà visitée
                if self.is_already_visited(world_x, world_y):
                    continue
                
                # Calculer la distance au robot
                distance = math.sqrt(
                    (centroid_col - robot_x_grid)**2 + 
                    (centroid_row - robot_y_grid)**2
                ) * resolution
                
                # Vérifier que la frontière n'est pas trop proche d'obstacles
                safety_check = self.check_safety(centroid_row, centroid_col, occupied, resolution)
                if not safety_check:
                    continue
                
                # Score basé sur la taille et la distance
                # Plus grande et plus proche = meilleur score
                size_score = math.log(frontier_size + 1)
                distance_score = 1.0 / (distance + 1.0)
                score = size_score * distance_score
                
                if score > best_score:
                    best_score = score
                    best_frontier = (world_x, world_y)
                    self.get_logger().info(
                        f'   Frontière #{i}: taille={frontier_size}, '
                        f'distance={distance:.2f}m, score={score:.3f}'
                    )
            
            if best_frontier:
                self.visited_frontiers.append(best_frontier)
                self.get_logger().info(f'✨ Meilleure frontière sélectionnée: {best_frontier}')
            
            return best_frontier
            
        except Exception as e:
            self.get_logger().error(f'❌ Erreur dans find_best_frontier: {e}')
            return None
    
    def check_safety(self, row, col, occupied_map, resolution):
        """Vérifie qu'une position est sûre (pas trop proche d'obstacles)"""
        safety_cells = int(self.safety_distance / resolution)
        
        row_min = max(0, int(row) - safety_cells)
        row_max = min(occupied_map.shape[0], int(row) + safety_cells)
        col_min = max(0, int(col) - safety_cells)
        col_max = min(occupied_map.shape[1], int(col) + safety_cells)
        
        region = occupied_map[row_min:row_max, col_min:col_max]
        
        # Retourner False si des obstacles sont trop proches
        return not np.any(region)
    
    def is_already_visited(self, x, y, threshold=1.5):
        """Vérifie si une position a déjà été visitée"""
        for visited_x, visited_y in self.visited_frontiers:
            distance = math.sqrt((x - visited_x)**2 + (y - visited_y)**2)
            if distance < threshold:
                return True
        return False
    
    def navigate_to_point(self, point):
        """Envoie le robot vers un point"""
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('⚠️  Serveur de navigation non disponible')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = point[0]
        goal_msg.pose.pose.position.y = point[1]
        goal_msg.pose.pose.position.z = 0.0
        
        # Calculer l'orientation vers la cible
        if self.robot_pose:
            dx = point[0] - self.robot_pose.position.x
            dy = point[1] - self.robot_pose.position.y
            yaw = math.atan2(dy, dx)
            
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2)
        else:
            goal_msg.pose.pose.orientation.w = 1.0
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback pour la réponse du goal"""
        try:
            self.goal_handle = future.result()
            if not self.goal_handle.accepted:
                self.get_logger().warn('⚠️  Goal rejeté par Nav2')
                return
            
            self.get_logger().info('✅ Goal accepté - Navigation en cours...')
        except Exception as e:
            self.get_logger().error(f'❌ Erreur goal response: {e}')
    
    def rotate_in_place(self):
        """Fait tourner le robot sur place pour scanner"""
        twist = Twist()
        twist.angular.z = 0.5  # rad/s
        
        # Rotation complète (2π / 0.5 ≈ 12.5 secondes)
        duration = 2 * math.pi / abs(twist.angular.z)
        rate = 10  # Hz
        
        for _ in range(int(duration * rate)):
            self.cmd_vel_pub.publish(twist)
            time.sleep(1.0 / rate)
        
        # Arrêter
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        time.sleep(0.5)
    
    def stop_exploration(self):
        """Arrête l'exploration"""
        self.exploring = False
        self.exploration_complete = True
        
        # Arrêter le robot
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
        
        # Annuler le goal en cours
        if self.goal_handle:
            try:
                self.goal_handle.cancel_goal_async()
            except:
                pass
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('🎉 EXPLORATION TERMINÉE !')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'📊 Statistiques:')
        self.get_logger().info(f'   - Frontières explorées: {len(self.visited_frontiers)}')
        self.get_logger().info(f'   - Temps écoulé: {int(time.time() - self.start_time)}s')
        self.get_logger().info('=' * 60)
        self.get_logger().info('💾 Pour sauvegarder la carte, exécutez:')
        self.get_logger().info('   mkdir -p ~/maps')
        self.get_logger().info('   ros2 run nav2_map_server map_saver_cli -f ~/maps/cafe_map')
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    
    explorer = AutonomousExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.get_logger().info('🛑 Exploration interrompue par l\'utilisateur')
    finally:
        explorer.stop_exploration()
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()