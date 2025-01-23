# Fichier **`drive_node.py`**

Le fichier **`drive_node.py`** définit un nœud ROS2 qui publie des commandes de mouvement sous forme de messages **`Twist`** pour contrôler un robot. Le nœud publie régulièrement des commandes de vitesse linéaire et angulaire afin de faire avancer le robot. Ce fichier est utile pour des simulations ou des scénarios où le robot doit se déplacer en ligne droite à une vitesse constante sans rotation.

---

## **Contenu du fichier `drive_node.py`**

### **1. Déclaration des importations**
- **`rclpy`** : Le module principal de ROS2 pour travailler avec des nœuds. Il permet de créer et gérer des nœuds ROS.
- **`geometry_msgs.msg.Twist`** : Importation du message **`Twist`**, qui contient les informations nécessaires pour commander la vitesse linéaire et angulaire d'un robot.

### **2. Classe `DriveNode`**
- La classe **`DriveNode`** hérite de **`Node`** et crée un éditeur pour publier des messages **`Twist`** sur le topic **`cmd_vel`**.
- **`__init__`** : Le constructeur du nœud initialise la publication sur le topic **`cmd_vel`**, ainsi qu'un timer pour publier un message toutes les 0.5 secondes.
- **`timer_callback`** : Cette méthode est appelée toutes les 0.5 secondes par le timer. Elle définit la vitesse linéaire à 10.0 m/s sur l'axe **`x`** et la vitesse angulaire à 0.0 sur l'axe **`z`** (mouvement droit sans rotation). Ensuite, elle publie le message **`cmd_vel_msg`** sur le topic **`cmd_vel`**.

### **3. Fonction `main`**
- La fonction **`main`** initialise le nœud ROS2, crée une instance du nœud **`DriveNode`**, et met en fonctionnement le nœud en appelant **`rclpy.spin()`**. Cela permet au nœud de rester actif et de publier régulièrement des commandes de mouvement.
- Après l'exécution, le nœud est détruit et **`rclpy.shutdown()`** est appelé pour arrêter le système ROS2 proprement.

---

## **Résumé du fichier `drive_node.py`**

Le fichier **`drive_node.py`** est un script Python ROS2 qui crée un nœud responsable de la publication de commandes de mouvement pour un robot. Le nœud publie des messages **`Twist`** sur le topic **`cmd_vel`** toutes les 0.5 secondes. Ces messages définissent la vitesse linéaire du robot sur l'axe **`x`** (avant) et la vitesse angulaire sur l'axe **`z`** (aucune rotation). Le fichier est principalement utilisé pour simuler des commandes simples de déplacement pour un robot dans un environnement ROS2.

---
