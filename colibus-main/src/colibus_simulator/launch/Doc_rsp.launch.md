# Fichier **rsp.launch.py** du projet **colibus_simulator**

Le fichier **rsp.launch.py** est utilisé pour configurer et lancer le nœud **`robot_state_publisher`** dans un environnement ROS2. Il génère la description du robot en utilisant un fichier **URDF** (Unified Robot Description Format) transformé via **xacro** et permet d'activer l'utilisation de l'horloge simulée et de **ros2_control** pour le contrôle du robot dans la simulation.

---

## **Contenu du fichier `rsp.launch.py`**

### **1. Déclaration des importations**
- **`os`** : Utilisé pour interagir avec le système de fichiers, en particulier pour obtenir les chemins des fichiers.
- **`ament_index_python.packages`** : Permet d'obtenir le chemin du répertoire de partage d'un package ROS2.
- **`launch`** : Contient des classes et des fonctions pour créer des descriptions de lancement, déclarer des arguments et configurer des nœuds.
- **`launch_ros.actions.Node`** : Permet de définir et de lancer des nœuds ROS2 dans une description de lancement.
- **`xacro`** : Utilisé pour générer un fichier URDF dynamique à partir de fichiers **xacro** (XML Macro Language). C'est un moyen de paramétrer et de générer des fichiers URDF complexes de manière flexible.
- **`launch.substitutions.Command`** : Permet d'exécuter des commandes système dans une description de lancement.

### **2. Déclaration des configurations et paramètres**
- **`use_sim_time`** : Un paramètre de lancement qui permet d'utiliser l'horloge simulée (Gazebo). Si ce paramètre est défini sur `true`, la simulation utilise l'horloge de simulation.
- **`use_ros2_control`** : Un paramètre de lancement qui permet d'activer **ros2_control**, un système de gestion des contrôles matériels pour les robots dans ROS2.

### **3. Traitement du fichier URDF**
- **`pkg_path`** : Le chemin du répertoire du package **colibus_simulator**, obtenu via **`get_package_share_directory`**.
- **`xacro_file`** : Le chemin vers le fichier **URDF** du robot, qui est un fichier **xacro** situé dans le répertoire **urdf** du package.
- **`robot_description_config`** : Utilise la commande **`xacro`** pour transformer le fichier **URDF** en une description dynamique du robot, avec les paramètres **use_ros2_control** et **use_sim_time** transmis via des substitutions.

### **4. Création du nœud `robot_state_publisher`**
- **`params`** : Un dictionnaire qui contient les paramètres à passer au nœud **`robot_state_publisher`**. Il inclut la description du robot **`robot_description`** générée à partir du fichier **URDF** et les paramètres de simulation (**use_sim_time**).
- **`node_robot_state_publisher`** : Déclare un nœud **`robot_state_publisher`** qui est responsable de la publication des transformations (TF) entre les différentes parties du robot, en utilisant la description du robot générée.

### **5. Description du lancement**
- **`LaunchDescription`** : Crée une description de lancement qui regroupe toutes les actions à exécuter lors de l'exécution du fichier de lancement.
  - **`DeclareLaunchArgument`** : Déclare les arguments de lancement **`use_sim_time`** et **`use_ros2_control`**. Cela permet de spécifier si l'horloge de simulation et **ros2_control** doivent être activés.
  - **`node_robot_state_publisher`** : Ajoute le nœud **`robot_state_publisher`** à la description de lancement.

---

## **Résumé du fichier `rsp.launch.py`**

Le fichier **rsp.launch.py** est utilisé pour démarrer le nœud **`robot_state_publisher`** qui publie les transformations du robot dans le système ROS2 en utilisant une description générée à partir d'un fichier **xacro**.

Les principales étapes de ce fichier sont :
1. Déclaration des paramètres **`use_sim_time`** et **`use_ros2_control`**.
2. Traitement du fichier **xacro** pour générer une description du robot (**URDF**).
3. Lancement du nœud **`robot_state_publisher`** avec la description du robot et les paramètres appropriés.
