# Fichier **`Auto_Drive.py`**

Le fichier **`Auto_Drive.py`** définit un nœud ROS2 permettant de contrôler un robot autonome à l'aide d'une caméra pour afficher une vue en temps réel et une vitesse constante. Ce nœud publie des commandes de mouvement (vitesse linéaire et angulaire) pour piloter le robot et affiche les informations de vitesse et de direction sur les images capturées par la caméra.

---

## **Contenu du fichier `Auto_Drive.py`**

### **1. Déclaration des importations**
- **`rclpy`** : Utilisé pour la gestion du nœud ROS2 et la communication.
- **`Node`** : La classe de base pour créer un nœud ROS2.
- **`sensor_msgs.msg.Image`** : Pour recevoir des messages d'images de la caméra.
- **`cv_bridge`** : Utilisé pour convertir les messages d'images ROS en images OpenCV.
- **`cv2`** : Bibliothèque OpenCV pour afficher les images et manipuler les images.
- **`geometry_msgs.msg.Twist`** : Pour publier des messages contenant des commandes de mouvement du robot (vitesse linéaire et angulaire).

### **2. Classe `AutoDriveNode`**
La classe **`AutoDriveNode`** est un nœud ROS2 qui reçoit les images de la caméra et publie des commandes de mouvement pour piloter le robot.

- **`__init__`** : Initialise le nœud, les abonnements, la publication, et le timer pour publier des messages de commande de mouvement à intervalles réguliers.
  - **`self.publisher_`** : Publie des messages de type **`Twist`** sur le topic **`/cmd_vel`** pour contrôler le mouvement du robot.
  - **`self.subscription`** : S'abonne au topic **`/camera/image_raw`** pour recevoir les images de la caméra.
  - **`self.bridge`** : Utilisé pour convertir les messages d'image ROS en images OpenCV.
  - **`self.timer`** : Crée un timer qui appelle la fonction **`publish_twist`** toutes les 0.1 secondes pour envoyer des commandes de mouvement.

### **3. Méthodes principales**
- **`image_callback(self, msg)`** : Cette méthode est appelée chaque fois qu'une nouvelle image est reçue du topic **`/camera/image_raw`**.
  - Convertit l'image ROS en image OpenCV.
  - Ajoute des informations de vitesse et de direction sur l'image.
  - Affiche l'image avec les annotations à l'aide de **OpenCV**.

- **`publish_twist(self)`** : Cette méthode est appelée par le timer pour publier une commande de mouvement à intervalles réguliers.
  - Crée un message **`Twist`** avec une vitesse linéaire constante (0.2 m/s) et une vitesse angulaire nulle (pour aller tout droit).
  - Publie ce message sur le topic **`/cmd_vel`** pour contrôler le robot.

### **4. Fonction `main`**
La fonction **`main`** initialise le nœud ROS2 et lance la boucle d'exécution du nœud avec **`rclpy.spin()`**. Elle permet également de gérer proprement l'arrêt du nœud en cas d'interruption clavier (**`KeyboardInterrupt`**), et de nettoyer les ressources, notamment en fermant les fenêtres OpenCV.

### **5. Fonctionnalités de débogage et d'affichage**
- L'image de la caméra est affichée dans une fenêtre OpenCV intitulée **`Camera Feed`**.
- Les informations de vitesse et de direction sont superposées sur l'image en temps réel, permettant de voir l'état du robot.
  
---

## **Résumé du fichier `Auto_Drive.py`**

Le fichier **`Auto_Drive.py`** définit un nœud ROS2 **`AutoDriveNode`** qui utilise la caméra du robot pour afficher les images en temps réel et y superposer des informations sur la vitesse et la direction du robot. Il publie des commandes de mouvement à une fréquence régulière, commandant une vitesse linéaire constante et une vitesse angulaire nulle, ce qui permet au robot de se déplacer en ligne droite. Ce fichier sert de base pour un robot autonome avec des commandes simples et un affichage en temps réel des informations liées à la conduite.

---

