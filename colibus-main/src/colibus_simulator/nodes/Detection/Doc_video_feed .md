# Fichier **`video_feed_in.py`**

Le fichier **`video_feed_in.py`** implémente un nœud ROS2 qui reçoit des flux d'images en provenance de plusieurs caméras et effectue un traitement en temps réel pour permettre une navigation autonome avec une assistance de conduite. Ce nœud gère également la publication des commandes de vitesse et d'angle pour un robot (ou un véhicule simulé), en fonction des images et des données de position fournies par un capteur LiDAR ou un système de navigation par satellite.

---

## **Contenu du fichier `video_feed_in.py`**

### **1. Déclaration des importations**
- **`cv2`** : Utilisé pour le traitement d'images avec OpenCV, permettant de manipuler les images reçues de la caméra.
- **`geometry_msgs.msg.Twist`** : Importation du message **`Twist`** pour envoyer des commandes de mouvement (linéaires et angulaires) à un robot.
- **`rclpy.node.Node`** : La classe de base pour créer un nœud ROS2.
- **`CvBridge`** : Permet de convertir des images ROS2 en images OpenCV.
- **`sensor_msgs.msg.Image`** : Utilisé pour recevoir des images sous forme de messages ROS2.
- **`nav_msgs.msg.Odometry`** : Permet de recevoir les informations d'odométrie, telles que la position et la vitesse du robot.
- **`matplotlib` et `FuncAnimation`** : Utilisés pour créer des graphiques animés, par exemple pour visualiser les données de conduite du robot.
- **`threading` et `concurrent.futures`** : Ces modules permettent de gérer les threads et d'exécuter des tâches en parallèle.

### **2. Classe `Video_feed_in`**
La classe **`Video_feed_in`** représente un nœud ROS2 qui gère les flux vidéo provenant de différentes caméras et les utilise pour la navigation du robot.
- **`__init__`** : Le constructeur initialise plusieurs abonnements (caméra, odométrie, etc.) et un éditeur pour publier des commandes de mouvement (**`cmd_vel`**).
  - Il crée des abonnements pour recevoir les flux d'images des caméras et la position du robot, ainsi qu'un abonnement pour recevoir des images de la caméra satellite pour la navigation.
  - Il configure un système d'animation pour afficher en temps réel les changements d'angle du robot.
  
- **`animate_prius_turning`** : Cette fonction est responsable de l'animation du volant du robot (ou de la voiture) et de l'affichage des angles de direction et de la moyenne mobile de ces angles.

- **`animate`** : Lance une animation en utilisant **`FuncAnimation`** de **`matplotlib`** pour visualiser le comportement de la direction du robot.

- **`process_sat_data`** : Cette fonction traite les données d'image reçues de la caméra satellite et les convertit en format exploitable pour la navigation.

- **`process_data`** : La fonction principale qui gère l'entrée des données d'image de la caméra et effectue la navigation autonome.
  - Le robot utilise les images de la caméra pour naviguer vers un "maison" spécifiée sur une carte virtuelle, similaire à la navigation par GPS.
  - La vitesse et la direction du robot sont déterminées en fonction des données du système de navigation par satellite et de la position actuelle du robot.
  - Les commandes de mouvement sont publiées sur le topic **`cmd_vel`** pour diriger le robot.
  - Si la navigation par satellite est activée, la vitesse du robot est influencée par cette dernière, et des ajustements de direction peuvent être appliqués pour éviter des obstacles ou tourner à un carrefour.

- **`main`** : La fonction principale qui initialise le nœud ROS2, lance les abonnements et gère l'animation du volant du robot si nécessaire. Le nœud est ensuite maintenu en fonctionnement avec **`rclpy.spin()`**.

### **3. Gestion de la vitesse et de la direction**
Les paramètres de vitesse et de direction du robot sont mis à jour en fonction de la simulation et de l'activation du système de navigation par satellite. Si le système de navigation par satellite est activé, il influence la direction du robot en cas de virages serrés. En mode normal, la vitesse est contrôlée par des commandes manuelles ou d'autres systèmes de navigation.

### **4. Détails supplémentaires**
- **Animation du volant** : L'animation est utilisée pour afficher les changements dans la direction du robot. Elle utilise les angles de direction mesurés et calcule une moyenne mobile pour lisser le comportement du volant.
- **Affichage des images** : Si la navigation par satellite est désactivée, les images capturées par la caméra sont affichées à l'utilisateur via OpenCV.
- **Débogage** : Un mode de débogage est disponible pour suivre le nombre de threads actifs, ce qui permet de surveiller l'exécution en parallèle des différentes fonctions.

---

## **Résumé du fichier `video_feed_in.py`**

Le fichier **`video_feed_in.py`** implémente un nœud ROS2 permettant de recevoir et de traiter des flux d'images provenant de plusieurs caméras et de la position du robot. Le nœud utilise ces données pour effectuer une navigation autonome en envoyant des commandes de mouvement. Il intègre également des fonctionnalités d'animation pour visualiser la direction du robot et de débogage pour surveiller l'exécution du système.

---
