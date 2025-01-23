# Fichier **launch_robot.launch.py** du projet **colibus_simulator**

Le fichier **launch_robot.launch.py** est responsable du lancement de l'ensemble des composants nécessaires pour faire fonctionner un robot dans le cadre du projet **colibus_simulator**. Il inclut divers fichiers de lancement et initialise plusieurs nœuds nécessaires à l'intégration des différents éléments du robot, comme le **robot_state_publisher**, les contrôleurs et le multiplexeur de commandes.

---

## **Contenu du fichier `launch_robot.launch.py`**

### **1. Déclaration des importations**
- **`os`** : Permet d'interagir avec le système de fichiers, en particulier pour localiser les fichiers de configuration et de lancement.
- **`ament_index_python.packages`** : Utilisé pour récupérer le chemin du répertoire de partage du package.
- **`launch`** : Contient les fonctionnalités de base pour le lancement de nœuds, l'inclusion de fichiers de lancement, et la gestion des événements.
- **`launch_ros.actions.Node`** : Permet de définir et de lancer des nœuds ROS2 dans le cadre du fichier de lancement.
- **`launch.launch_description_sources.PythonLaunchDescriptionSource`** : Permet d'inclure d'autres fichiers de lancement Python dans la description du lancement.
- **`launch.substitutions.Command`** : Permet d'exécuter une commande shell dans le cadre du lancement.
- **`launch.actions.TimerAction`** : Permet de retarder l'exécution d'une action, utilisée pour lancer des nœuds avec un délai.
- **`launch.actions.RegisterEventHandler`** et **`launch.event_handlers.OnProcessStart`** : Utilisés pour gérer des événements spécifiques lors du démarrage des nœuds (par exemple, lancer un autre nœud lorsque le contrôleur est démarré).

### **2. Initialisation des différents nœuds et actions**

#### **`rsp` (Robot State Publisher)**
- Ce nœud est responsable de publier l'état du robot sur le topic `/robot_state_publisher`. Il utilise le fichier `rsp.launch.py` du package `articubot_one`.
  - **Argument de lancement :**
    - **`use_sim_time`** : Définit si le robot utilise le temps simulé (`false`).
    - **`use_ros2_control`** : Permet d'utiliser le système de contrôle ROS2 (`true`).

#### **`twist_mux`**
- **Package :** `twist_mux`
- **Exécutable :** `twist_mux`
- **Paramètres :**
  - **`twist_mux.yaml`** : Fichier de configuration pour le multiplexeur de commandes de mouvement du robot.
- **Remapping :**
  - **`/cmd_vel_out` → `/diff_cont/cmd_vel_unstamped`** : Le topic de commande de vitesse du robot est remappé.

#### **`robot_description`**
- Utilise une commande ROS2 pour récupérer la description du robot (`robot_description`) et la passer au nœud `controller_manager`.
  - **Commande :** `ros2 param get --hide-type /robot_state_publisher robot_description`

#### **`controller_manager`**
- **Package :** `controller_manager`
- **Exécutable :** `ros2_control_node`
- **Paramètres :**
  - **`robot_description`** : La description du robot.
  - **`controller_params_file`** : Le fichier `my_controllers.yaml` qui contient la configuration des contrôleurs du robot.

#### **`delayed_controller_manager`**
- Utilise un délai de 3 secondes avant de démarrer le `controller_manager` via **`TimerAction`**.

#### **`diff_drive_spawner`**
- **Package :** `controller_manager`
- **Exécutable :** `spawner.py`
- **Arguments :**
  - **`diff_cont`** : Spécifie le contrôleur du robot à utiliser pour la conduite différenciée (mouvement du robot).

#### **`joint_broad_spawner`**
- **Package :** `controller_manager`
- **Exécutable :** `spawner.py`
- **Arguments :**
  - **`joint_broad`** : Spécifie le contrôleur pour la gestion des articulations du robot (par exemple, les roues ou autres joints).

### **3. Gestion des événements**
- **`delayed_diff_drive_spawner`** : Ce gestionnaire d'événements démarre le spawner pour le contrôleur `diff_cont` lorsque le `controller_manager` commence à s'exécuter.
- **`delayed_joint_broad_spawner`** : Ce gestionnaire d'événements démarre le spawner pour le contrôleur `joint_broad` de la même manière.

### **4. Lancement des nœuds**
Le fichier de lancement retourne une **`LaunchDescription`** qui inclut tous les nœuds et actions définis précédemment :
- **`rsp`** : Lance le `robot_state_publisher`.
- **`twist_mux`** : Lance le multiplexeur des commandes de vitesse.
- **`delayed_controller_manager`** : Lance le `controller_manager` après un délai de 3 secondes.
- **`delayed_diff_drive_spawner`** : Lance le contrôleur différentiel après le démarrage du `controller_manager`.
- **`delayed_joint_broad_spawner`** : Lance le contrôleur des articulations après le démarrage du `controller_manager`.

---


