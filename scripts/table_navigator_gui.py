#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml
import tkinter as tk
from tkinter import ttk, messagebox
import threading

class TableNavigatorGUI(Node):
    def __init__(self):
        super().__init__('table_navigator_gui')
        
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.table_positions = self.load_table_positions()
        self.current_goal_handle = None
        
        self.get_logger().info('Table Navigator GUI initialisé')
    
    def load_table_positions(self):
        """Charge les positions des tables"""
        try:
            config_file = '/home/ros2_ws_project/src/ros2-slam-auto-navigation/config/table_positions.yaml'
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                return config['tables']
        except:
            return {
                'table_1': {
                    'position': {'x': 2.5, 'y': 3.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                    'description': 'Table principale'
                },
                'table_2': {
                    'position': {'x': -1.5, 'y': 2.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.707, 'w': 0.707},
                    'description': 'Table près de la fenêtre'
                },
                'table_3': {
                    'position': {'x': 0.0, 'y': -2.5, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 1.0, 'w': 0.0},
                    'description': 'Table du fond'
                },
                'comptoir': {
                    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                    'description': 'Point de départ / Comptoir'
                }
            }
    
    def navigate_to_table(self, table_name, status_label):
        """Navigation vers une table"""
        if table_name not in self.table_positions:
            messagebox.showerror("Erreur", f"Table '{table_name}' non trouvée!")
            return
        
        status_label.config(text=f"Navigation vers {table_name}...", fg="blue")
        
        # Attendre le serveur
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            status_label.config(text="Erreur: Serveur non disponible", fg="red")
            messagebox.showerror("Erreur", "Le serveur de navigation n'est pas disponible!")
            return
        
        # Créer le goal
        table_pos = self.table_positions[table_name]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = table_pos['position']['x']
        goal_msg.pose.pose.position.y = table_pos['position']['y']
        goal_msg.pose.pose.position.z = table_pos['position']['z']
        
        goal_msg.pose.pose.orientation.x = table_pos['orientation']['x']
        goal_msg.pose.pose.orientation.y = table_pos['orientation']['y']
        goal_msg.pose.pose.orientation.z = table_pos['orientation']['z']
        goal_msg.pose.pose.orientation.w = table_pos['orientation']['w']
        
        # Envoyer le goal
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(
            lambda future: self.goal_response_callback(future, table_name, status_label)
        )
    
    def goal_response_callback(self, future, table_name, status_label):
        """Callback pour la réponse du goal"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            status_label.config(text="Goal rejeté!", fg="red")
            return
        
        self.current_goal_handle = goal_handle
        status_label.config(text=f"En route vers {table_name}...", fg="green")
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.result_callback(future, table_name, status_label)
        )
    
    def result_callback(self, future, table_name, status_label):
        """Callback pour le résultat final"""
        status_label.config(text=f"Arrivé à {table_name}!", fg="green")
        messagebox.showinfo("Succès", f"Robot arrivé à {table_name}!")
    
    def cancel_navigation(self, status_label):
        """Annule la navigation en cours"""
        if self.current_goal_handle:
            self.current_goal_handle.cancel_goal_async()
            status_label.config(text="Navigation annulée", fg="orange")
            self.current_goal_handle = None

def create_gui(navigator):
    """Crée l'interface graphique"""
    root = tk.Tk()
    root.title("🤖 Navigation Robot - Service Café")
    root.geometry("500x600")
    root.configure(bg='#f0f0f0')
    
    # Style
    style = ttk.Style()
    style.theme_use('clam')
    
    # Titre
    title_frame = tk.Frame(root, bg='#2c3e50', height=80)
    title_frame.pack(fill=tk.X, pady=(0, 20))
    
    title_label = tk.Label(
        title_frame,
        text="☕ Robot Serveur de Café",
        font=('Arial', 20, 'bold'),
        bg='#2c3e50',
        fg='white'
    )
    title_label.pack(pady=20)
    
    # Frame principal
    main_frame = tk.Frame(root, bg='#f0f0f0')
    main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=10)
    
    # Instructions
    instructions = tk.Label(
        main_frame,
        text="Sélectionnez une table pour livrer la commande",
        font=('Arial', 12),
        bg='#f0f0f0',
        fg='#34495e'
    )
    instructions.pack(pady=10)
    
    # Frame pour les boutons de tables
    tables_frame = tk.Frame(main_frame, bg='#f0f0f0')
    tables_frame.pack(pady=20, fill=tk.BOTH, expand=True)
    
    # Statut
    status_label = tk.Label(
        main_frame,
        text="En attente...",
        font=('Arial', 11),
        bg='#f0f0f0',
        fg='gray'
    )
    status_label.pack(pady=20)
    
    # Créer un bouton pour chaque table
    for idx, (table_name, table_info) in enumerate(navigator.table_positions.items()):
        btn_frame = tk.Frame(tables_frame, bg='white', relief=tk.RAISED, borderwidth=2)
        btn_frame.pack(fill=tk.X, pady=8, padx=10)
        
        # Emoji basé sur le type de table
        emoji = "🏠" if 'comptoir' in table_name.lower() else "🪑"
        
        btn = tk.Button(
            btn_frame,
            text=f"{emoji} {table_name.replace('_', ' ').title()}",
            font=('Arial', 14, 'bold'),
            bg='#3498db',
            fg='white',
            activebackground='#2980b9',
            activeforeground='white',
            relief=tk.FLAT,
            cursor='hand2',
            command=lambda tn=table_name: navigator.navigate_to_table(tn, status_label)
        )
        btn.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Description si disponible
        if 'description' in table_info:
            desc_label = tk.Label(
                btn_frame,
                text=table_info['description'],
                font=('Arial', 9),
                bg='white',
                fg='#7f8c8d'
            )
            desc_label.pack(side=tk.LEFT, padx=5)
    
    # Bouton d'annulation
    cancel_btn = tk.Button(
        main_frame,
        text="❌ Annuler la navigation",
        font=('Arial', 12),
        bg='#e74c3c',
        fg='white',
        activebackground='#c0392b',
        activeforeground='white',
        relief=tk.FLAT,
        cursor='hand2',
        command=lambda: navigator.cancel_navigation(status_label)
    )
    cancel_btn.pack(pady=10, fill=tk.X, padx=10)
    
    # Info
    info_label = tk.Label(
        root,
        text="Assurez-vous que la simulation et Nav2 sont lancés",
        font=('Arial', 9),
        bg='#f0f0f0',
        fg='#95a5a6'
    )
    info_label.pack(pady=10)
    
    return root

def ros_spin(navigator):
    """Thread pour faire tourner le nœud ROS"""
    rclpy.spin(navigator)

def main(args=None):
    rclpy.init(args=args)
    
    navigator = TableNavigatorGUI()
    
    # Créer un thread pour ROS
    ros_thread = threading.Thread(target=ros_spin, args=(navigator,), daemon=True)
    ros_thread.start()
    
    # Créer et lancer la GUI
    root = create_gui(navigator)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()