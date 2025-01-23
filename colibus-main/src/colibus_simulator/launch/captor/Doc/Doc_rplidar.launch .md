# Fichier **rplidar.launch.py**

Le fichier **`rplidar.launch.py`** est utilisé pour configurer et démarrer un nœud **rplidar_composition** du paquet **`rplidar_ros`** dans un environnement ROS2. Ce nœud est responsable de la communication avec un **LiDAR (Light Detection and Ranging)**, ici spécifiquement un capteur **RPLIDAR**, pour collecter des données de distance sous forme de nuages de points laser.

---

## **Contenu du fichier `rplidar.launch.py`**

### **1. Déclaration des importations**
- **`os`** : Utilisé pour interagir avec le système de fichiers (bien qu'il ne soit pas utilisé directement dans ce fichier, il est souvent inclus pour des raisons d'extension ou de configuration dynamique).
- **`launch`** : Fournit les mécanismes nécessaires pour définir des descriptions de lancement dans ROS2.
- **`launch_ros.actions.Node`** : Utilisé pour créer un nœud ROS2 dans une description de lancement. C'est ici qu'on configure et lance le nœud **rplidar_composition**.

### **2. Fonction `generate_launch_description`**
- **`LaunchDescription`** : Cette fonction crée une **`LaunchDescription`** dans laquelle les actions de lancement sont définies. Ici, l'action est la création et le lancement du nœud **`rplidar_composition`**.
  - **`Node`** : Le nœud **`rplidar_composition`** appartient au paquet **`rplidar_ros`** et est exécuté pour permettre la communication avec le capteur RPLIDAR.
  - Le nœud reçoit plusieurs paramètres importants :
    - **`serial_port`** : Définit le port série utilisé pour la communication avec le RPLIDAR. Dans ce cas, il s'agit du chemin du périphérique **`/dev/serial/by-path/`**, qui peut varier en fonction du système et du périphérique connecté.
    - **`frame_id`** : Définit le nom du cadre de référence de la mesure LiDAR. Ici, il est nommé **`laser_frame`**, ce qui signifie que le cadre de la mesure LiDAR est associé à ce nom.
    - **`angle_compensate`** : Ce paramètre est défini sur **`True`**, ce qui signifie que les compensations d'angle seront appliquées lors de la lecture des données du LiDAR.
    - **`scan_mode`** : Définit le mode de balayage. Ici, **`Standard`** est utilisé, ce qui signifie que le LiDAR effectuera un balayage standard pour collecter des données de distance.

### **3. Détails de l'exécution**
- Le nœud **`rplidar_composition`** est exécuté avec les paramètres définis, et les données collectées à partir du capteur LiDAR sont envoyées à la sortie **`screen`** pour affichage dans le terminal lors de l'exécution.
- Les paramètres sont définis dans une liste **`parameters`**, ce qui permet de personnaliser la configuration du capteur en fonction des besoins.

---

## **Résumé du fichier `rplidar.launch.py`**

Le fichier **`rplidar.launch.py`** est utilisé pour démarrer le nœud **`rplidar_composition`** du paquet **`rplidar_ros`**. Ce nœud gère la communication avec un capteur RPLIDAR pour collecter des données LiDAR, comme la distance et la position des objets autour du robot. Les paramètres de lancement incluent le port série du LiDAR, l'identifiant du cadre de référence des données LiDAR, ainsi que d'autres paramètres tels que la compensation d'angle et le mode de balayage.


---

