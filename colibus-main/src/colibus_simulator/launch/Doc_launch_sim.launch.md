# Fichier **launch_sim.launch.py** du projet **colibus_simulator**

Le fichier **launch_sim.launch.py** est responsable du lancement de tous les composants nécessaires pour faire fonctionner le robot dans un environnement simulé avec Gazebo. Il inclut des fichiers de lancement pour le **robot_state_publisher**, le **joystick**, le **twist_mux** pour gérer les commandes de mouvement, ainsi que la configuration de **Gazebo** pour la simulation et l'initialisation des contrôleurs.

---

## **Contenu du fichier `launch_sim.launch.py`**

### **1. Déclaration des importations**
- **`os`** : Permet d'interagir avec le système de fichiers pour localiser les fichiers nécessaires au lancement et à la configuration.
- **`ament_index_python.packages`** : Utilisé pour obtenir le chemin du répertoire de partage du package ROS2.
- **`launch`** : Contient les fonctionnalités nécessaires pour la création et l'exécution de la description du lancement (inclus des fichiers de lancement, lancement de nœuds, gestion des événements).
- **`launch_ros.actions.Node`** : Permet de définir et de lancer des nœuds ROS2 dans la description du lancement.
- **`launch.launch_description_sources.PythonLaunchDescriptionSource`** : Permet d'inclure d'autres fichiers de lancement Python dans la description du lancement.

### **2. Initialisation des différents nœuds et actions**

#### **`rsp` (Robot State Publisher)**
- Ce nœud est responsable de publier l'état du robot. Il utilise le fichier `rsp.launch.py` du package **colibus_simulator**.
  - **Arguments de lancement** :
    - **`use_sim_time`** : Définit si le robot utilise le temps simulé (`true`).
    - **`use_ros2_control`** : Permet d'utiliser le système de contrôle ROS2 (`true`).

#### **`joystick`**
- Ce nœud permet de recevoir les commandes du joystick. Il inclut le fichier `joystick.launch.py` du package **colibus_simulator** pour l'initialisation du joystick.
  - **Argument de lancement** :
    - **`use_sim_time`** : Définit si le robot utilise le temps simulé (`true`).

#### **`twist_mux`**
- **Package** : `twist_mux`
- **Exécutable** : `twist_mux`
- **Paramètres** :
  - **`twist_mux.yaml`** : Fichier de configuration du multiplexeur des commandes de mouvement du robot.
  - **`use_sim_time`** : Définit si le robot utilise le temps simulé (`true`).
- **Remapping** :
  - **`/cmd_vel_out` → `/diff_cont/cmd_vel_unstamped`** : Le topic de commande de vitesse du robot est remappé.

#### **`gazebo`**
- Ce nœud inclut le fichier de lancement pour Gazebo à partir du package **gazebo_ros** et utilise le fichier de paramètres **`gazebo_params.yaml`** pour configurer Gazebo.
  - **Arguments de lancement** :
    - **`extra_gazebo_args`** : Permet d'ajouter des arguments supplémentaires à Gazebo, comme le fichier de paramètres.

#### **`spawn_entity`**
- **Package** : `gazebo_ros`
- **Exécutable** : `spawn_entity.py`
- **Arguments** :
  - **`-topic`** : Le topic à partir duquel l'entité robot est récupérée (`robot_description`).
  - **`-entity`** : Nom de l'entité robot à générer dans Gazebo (`my_bot`).

#### **`diff_drive_spawner`**
- **Package** : `controller_manager`
- **Exécutable** : `spawner`
- **Arguments** :
  - **`diff_cont`** : Le contrôleur pour la conduite différenciée du robot.

#### **`joint_broad_spawner`**
- **Package** : `controller_manager`
- **Exécutable** : `spawner`
- **Arguments** :
  - **`joint_broad`** : Le contrôleur pour la gestion des articulations du robot (par exemple, les roues ou autres joints).

### **3. Gestion des événements**
Le fichier inclut un exemple de gestion des événements avec **`RegisterEventHandler`** et **`OnProcessExit`**, bien que ce code soit commenté. L'idée est de retarder l'exécution du spawner pour le contrôleur différentiel jusqu'à ce que l'entité robot soit effectivement spawnée dans Gazebo.

#### **`delayed_diff_drive_spawner`** (commenté)
- Utilise **`OnProcessExit`** pour démarrer le spawner pour le contrôleur différentiel après que le nœud **`spawn_entity`** a été exécuté avec succès. Ce comportement est actuellement commenté dans le fichier.

### **4. Lancement des nœuds**
Le fichier de lancement retourne une **`LaunchDescription`** qui inclut tous les nœuds et actions définis précédemment :
- **`rsp`** : Lance le `robot_state_publisher`.
- **`joystick`** : Lance le nœud du joystick pour permettre le contrôle manuel du robot.
- **`twist_mux`** : Lance le multiplexeur des commandes de vitesse.
- **`gazebo`** : Lance la simulation Gazebo avec les paramètres fournis.
- **`spawn_entity`** : Spawne l'entité du robot dans Gazebo.
- **`diff_drive_spawner`** : Lance le contrôleur différentiel pour le robot.
- **`joint_broad_spawner`** : Lance le contrôleur des articulations du robot.

---

## **Résumé du fichier `launch_sim.launch.py`**

Le fichier **launch_sim.launch.py** est utilisé pour configurer et démarrer un environnement simulé avec Gazebo pour le robot. Il inclut plusieurs nœuds pour la simulation :
- **`robot_state_publisher`** pour publier l'état du robot.
- **`joystick`** pour recevoir des commandes de mouvement depuis un joystick.
- **`twist_mux`** pour gérer et multiplexeur les commandes de mouvement.
- **`gazebo`** pour initialiser la simulation Gazebo et spawnner le robot.
- **Les contrôleurs** nécessaires pour le mouvement différentiel et la gestion des articulations du robot.


---
