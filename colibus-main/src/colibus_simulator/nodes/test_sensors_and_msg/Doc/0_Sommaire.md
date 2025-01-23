# Sommaire des Fichiers

## 1. **`camera_front.py`**

Le fichier **`camera_front.py`** est un nœud ROS2 responsable de la gestion de la caméra avant d'un véhicule ou robot autonome. Ce nœud s'abonne aux images envoyées par la caméra et les publie sur un topic afin d'être utilisées par d'autres composants du système. Il inclut également un abonnement aux messages de la caméra pour effectuer un traitement ou des actions en fonction des images reçues.

### Fonctionnalités principales :
- Abonnement au topic `/upper_camera/image_raw` pour recevoir les images en temps réel.
- Publication des images traitées sous forme de frames vidéo.
- Enregistrement des frames vidéo dans un fichier `.avi`.

---

## 2. **`camera_rear.py`**

Le fichier **`camera_rear.py`** est similaire à **`camera_front.py`**, mais se concentre sur la gestion de la caméra arrière du véhicule. Ce nœud ROS2 s'abonne au topic des images de la caméra arrière et enregistre les images dans un fichier vidéo tout en les affichant à l'écran.

### Fonctionnalités principales :
- Abonnement au topic `/rear_camera/image_raw` pour recevoir les images en temps réel.
- Enregistrement des images dans un fichier `.avi`.
- Affichage des images reçues en temps réel.

---

## 3. **`vehicle_node.py`**

Le fichier **`vehicle_node.py`** représente un simulateur pour un véhicule autonome. Ce nœud ROS2 gère plusieurs aspects du véhicule, comme l'état de la voiture (vitesse, direction, freinage), l'état climatique, et les commandes des différents systèmes (freins, conduite en mode manuel ou automatique). Il expose des services permettant de contrôler différents modes du véhicule et de récupérer des informations sur son état.

### Fonctionnalités principales :
- Publication de l'état du véhicule, y compris la vitesse, l'accélération, et les commandes de direction.
- Publication de l'état climatique (température, direction de l'air, etc.).
- Exposition de services pour changer les modes de conduite (longitudinal et directionnel).
- Publication périodique d'erreurs de logs et d'identifiants de voiture simulés.

---

## 4. **`vlp16.py`**

Le fichier **`vlp16.py`** simule un système de capteurs pour un véhicule autonome, notamment un LiDAR (VLP-16), une caméra, un IMU (Unité de Mesure Inertielle) et un GPS. Ce nœud ROS2 publie des données simulées à des intervalles réguliers pour ces capteurs, ce qui permet de tester des algorithmes sans avoir besoin de matériel physique.

### Fonctionnalités principales :
- Publication de données simulées pour les capteurs : LiDAR, caméra, IMU, et GPS.
- Abonnements aux topics des capteurs pour recevoir des messages simulés.
- Publication à des intervalles réguliers pour chaque capteur simulé.
- Utilisation de valeurs aléatoires pour simuler les données des capteurs.

---

## **Conclusion**

Ces quatre fichiers **`camera_front.py`**, **`camera_rear.py`**, **`vehicle_node.py`**, et **`vlp16.py`** représentent des nœuds ROS2 utilisés pour simuler et gérer différents aspects d'un véhicule autonome. Ils permettent de tester et de développer des systèmes robotiques sans nécessiter de matériel physique. Chaque fichier a une fonctionnalité spécifique : gestion de caméra, simulation de véhicule et simulation de capteurs, avec des capacités de publication de données simulées pour effectuer des tests et des simulations en temps réel.
