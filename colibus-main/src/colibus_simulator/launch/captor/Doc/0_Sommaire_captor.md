# Sommaire des fichiers de lancement dans le dossier **launch/captor**

Le dossier **`launch/captor`** contient trois fichiers de lancement importants pour la simulation du robot, qui sont responsables de la gestion des capteurs et du contrôle du véhicule dans l'environnement ROS2. Voici un résumé des trois fichiers présents dans ce dossier.

---

## 1. **camera.launch.py**

Ce fichier lance un nœud pour la gestion d'une caméra utilisant le paquet **`v4l2_camera`**. Il configure un nœud **`v4l2_camera_node`** pour diffuser des images à partir d'une caméra en utilisant les paramètres spécifiés, tels que la taille de l'image et le taux de frames par seconde. Ce nœud est généralement utilisé pour obtenir des flux vidéo à des fins de perception dans des applications de robotique.

### **Points clés** :
- **Paquet** : `v4l2_camera`
- **Nœud** : `v4l2_camera_node`
- **Paramètres** : 
  - `image_size`: 640x480
  - `time_per_frame`: [1, 6]
  - `camera_frame_id`: `camera_link_optical`
  
---

## 2. **rplidar.launch.py**

Ce fichier de lancement configure un nœud pour le lidar **RPLIDAR** en utilisant le paquet **`rplidar_ros`**. Il lance le nœud **`rplidar_composition`**, qui est responsable de récupérer et de publier les données de télémétrie du lidar, telles que les scans laser. Le nœud est configuré avec des paramètres tels que le port série et le cadre du laser.

### **Points clés** :
- **Paquet** : `rplidar_ros`
- **Nœud** : `rplidar_composition`
- **Paramètres** :
  - `serial_port`: `/dev/serial/by-path/...`
  - `frame_id`: `laser_frame`
  - `angle_compensate`: `True`
  - `scan_mode`: `Standard`
  
---

## 3. **vehicle_simulator.launch.py**

Ce fichier lance le nœud **`vehicle_control_node`** du paquet **`colibus_simulator`**, qui est responsable du contrôle du véhicule dans la simulation. Ce nœud permet de gérer les actions du véhicule, telles que les déplacements dans un environnement simulé, et d'interagir avec d'autres éléments du système de simulation.

### **Points clés** :
- **Paquet** : `colibus_simulator`
- **Nœud** : `vehicle_control_node`
- **Sortie** : Les logs sont affichés sur l'écran pour le suivi en temps réel.



