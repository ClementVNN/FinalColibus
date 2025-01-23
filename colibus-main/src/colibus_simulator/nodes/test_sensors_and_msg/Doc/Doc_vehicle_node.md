# Fichier **`vehicle_node.py`**

Le fichier **`vehicle_node.py`** est un fichier ROS2 implémentant un simulateur de véhicule. Ce nœud ROS2 gère différents aspects du véhicule, tels que l'état du véhicule, la commande de la direction, l'accélération, les feux clignotants et les commandes des interfaces. Il publie des messages sur divers topics pour simuler l'état du véhicule et gérer les actions du véhicule en fonction des données reçues.

---

## **Contenu du fichier `vehicle_node.py`**

### **1. Importation des modules**
- **`rclpy`** : Le module nécessaire pour créer des nœuds ROS2.
- **`std_msgs.msg.Header, Bool`** : Importation de messages de base pour les en-têtes et les messages de type Booléen.
- **`colibus_simulator.msg`** : Messages personnalisés pour le simulateur de véhicule, incluant des informations sur l'état du véhicule, la climatisation, les PID (Proportional-Integral-Derivative), et d'autres paramètres de contrôle.
- **`colibus_simulator.srv`** : Services permettant de définir et obtenir des modes de contrôle longitudinal et de direction, ainsi que d'autres paramètres.

### **2. Classe `VehicleSimulatorNode`**
- **`VehicleSimulatorNode`** : Une classe héritée de **`Node`** qui représente le simulateur de véhicule. Ce nœud publie et reçoit des messages pour simuler un véhicule en temps réel.
  - **`self.car_status_pub`** : Un éditeur pour publier l'état du véhicule (vitesse, accélération, angle de direction, etc.).
  - **`self.clima_status_pub`** : Un éditeur pour publier l'état de la climatisation du véhicule.
  - **`self.steer_ang_pid_pub`, `self.brake_pid_pub`** : Éditeurs pour publier les paramètres PID pour la direction et les freins.
  - **`self.steering_data_pub`** : Éditeur pour publier les données de direction du véhicule.
  - **`self.speed_accel_data_pub`** : Éditeur pour publier les données de vitesse et d'accélération.
  - **`self.blk_data_sub`, `self.beep_data_sub`, `self.lcd_data_sub`** : Abonnements pour recevoir des messages sur les feux clignotants, les bips et le texte LCD.
  - **`self.set_auto_mode_sub`, `self.hand_brake_sub`** : Abonnements pour recevoir des messages sur le mode automatique et le frein à main.

### **3. Services**
- **`set_longitudinal_control_mode`** : Service permettant de définir le mode de contrôle longitudinal (pédale, accélération ou contrôle de la vitesse).
- **`get_longitudinal_control_mode`** : Service permettant d'obtenir le mode de contrôle longitudinal.
- **`set_steering_control_mode`** : Service permettant de définir le mode de contrôle de la direction (rapide, modéré, lent).
- **`get_steering_control_mode`** : Service permettant d'obtenir le mode de contrôle de la direction.
- **`get_pid_params`** : Service permettant d'obtenir les paramètres PID pour la direction et les freins.
- **`get_bl_status`** : Service permettant d'obtenir l'état des feux clignotants.
- **`get_run_mode`** : Service permettant d'obtenir le mode d'exécution du véhicule (normal, etc.).

### **4. Fonction `publish_topics`**
- **`publish_topics`** : Fonction qui publie régulièrement sur les différents topics du véhicule pour simuler l'état du véhicule, la climatisation, l'ID du véhicule, les erreurs de log, les PID et les données de vitesse et de direction.

### **5. Gestion des messages reçus**
- **`handle_blk_light`** : Gère les messages concernant les feux clignotants.
- **`handle_beep_data`** : Gère les messages concernant les bips (fréquence et durée).
- **`handle_lcd_text`** : Gère les messages concernant le texte affiché sur l'écran LCD.
- **`handle_hand_brake`** : Gère les messages concernant l'activation ou la désactivation du frein à main.
- **`handle_set_auto_mode`** : Gère les messages concernant la mise en mode automatique ou manuel.

### **6. Publication des messages**
- **`publish_car_status`** : Publie des informations aléatoires sur l'état du véhicule (vitesse, accélération, angle de direction, etc.).
- **`publish_clima_status`** : Publie des informations aléatoires sur l'état de la climatisation (température, vitesse du ventilateur, etc.).
- **`publish_car_id`** : Publie un ID aléatoire pour le véhicule et son nom.
- **`publish_log_err`** : Publie un message d'erreur de log simulé.
- **`publish_pid`** : Publie les paramètres PID simulés pour la direction ou les freins.
- **`publish_steering_data`** : Publie les données simulées de direction (accélération actuelle et de référence).
- **`publish_speed_accel_data`** : Publie les données simulées de vitesse et d'accélération (vitesse actuelle, accélération actuelle, etc.).

### **7. Fonction `main`**
- **`main`** : Fonction principale qui initialise le nœud ROS2 et le maintient en fonctionnement. Lors de l'exécution, le nœud attend les messages entrants et publie des informations sur l'état du véhicule à intervalles réguliers.

---

## **Résumé du fichier `vehicle_node.py`**

Le fichier **`vehicle_node.py`** est conçu pour simuler l'état d'un véhicule et publier ces informations sur différents topics ROS2. Il gère divers aspects du véhicule, y compris l'état de la voiture, la climatisation, les PID de contrôle, la direction, la vitesse et les feux clignotants. En plus de cela, il expose des services permettant de définir et obtenir des paramètres relatifs au mode de contrôle du véhicule (longitudinal et directionnel) ainsi que d'autres informations du véhicule.

---

