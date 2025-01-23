# Fichier **navigation_launch.py** du projet **articubot_one**

Le fichier **navigation_launch.py** est responsable du lancement des nœuds nécessaires pour la navigation du robot en utilisant la pile **Nav2**. Il configure et lance des nœuds tels que **`controller_server`**, **`planner_server`**, **`recoveries_server`**, **`bt_navigator`**, et **`waypoint_follower`**, tout en gérant le cycle de vie de ces nœuds. Ce fichier permet également de configurer des paramètres comme le fichier de comportement (**Behavior Tree**) et les paramètres de simulation.

---

## **Contenu du fichier `navigation_launch.py`**

### **1. Déclaration des importations**
- **`os`** : Utilisé pour interagir avec le système de fichiers, principalement pour accéder aux fichiers de configuration.
- **`ament_index_python.packages`** : Permet d'obtenir le chemin du répertoire de partage du package ROS2.
- **`launch`** : Contient des classes et des fonctions pour créer des descriptions de lancement, déclarer des arguments de lancement, et configurer les actions.
- **`launch_ros.actions.Node`** : Permet de définir et de lancer des nœuds ROS2 dans la description de lancement.
- **`nav2_common.launch.RewrittenYaml`** : Permet de manipuler et de réécrire des fichiers YAML avec des substitutions dans les paramètres de lancement.

### **2. Déclaration des variables et configurations**
- **`bringup_dir`** : Répertoire contenant les fichiers de configuration pour la navigation du robot, récupéré à partir du package **articubot_one**.
- **`namespace`** : Un paramètre de lancement optionnel permettant de définir un espace de noms pour les nœuds de navigation.
- **`use_sim_time`** : Paramètre permettant de définir si le robot doit utiliser le temps simulé (via Gazebo) ou le temps réel.
- **`autostart`** : Indique si la pile **Nav2** doit démarrer automatiquement.
- **`params_file`** : Le fichier YAML contenant les paramètres ROS2 utilisés pour configurer les nœuds de la pile **Nav2**.
- **`default_bt_xml_filename`** : Le chemin du fichier XML de l'arbre de comportements (**Behavior Tree**) utilisé pour la navigation.
- **`map_subscribe_transient_local`** : Paramètre indiquant si le robot doit utiliser un abonnement local et temporaire pour la carte.

### **3. Création des remappings**
Les remappings sont utilisés pour rediriger certains topics vers d'autres noms. Par exemple :
- **`/tf` → `tf`** : Remapping des topics pour les transformations.
- **`/tf_static` → `tf_static`** : Remapping pour les transformations statiques.

### **4. Configuration des paramètres via YAML**
Le fichier YAML contenant les paramètres pour la navigation est configuré avec des substitutions :
- **`use_sim_time`**, **`default_bt_xml_filename`**, **`autostart`**, et **`map_subscribe_transient_local`** sont des substitutions qui sont utilisées pour configurer la simulation, le fichier de comportement et l'abonnement à la carte.
- **`RewrittenYaml`** : Permet de réécrire le fichier de paramètres avec les substitutions et de charger les configurations de manière dynamique.

### **5. Définition des nœuds ROS2 à lancer**

#### **`controller_server`**
- **Package** : `nav2_controller`
- **Exécutable** : `controller_server`
- **Nom** : `controller_server`
- **Paramètres** : Le fichier de paramètres YAML réécrit avec les substitutions pour la configuration de la navigation.
- **Remappings** : Les topics de transformation sont remappés.

Ce nœud gère le contrôle du robot, en envoyant des commandes aux actionneurs pour suivre la trajectoire planifiée.

#### **`planner_server`**
- **Package** : `nav2_planner`
- **Exécutable** : `planner_server`
- **Nom** : `planner_server`
- **Paramètres** : Le même fichier de paramètres YAML pour configurer la planification des trajectoires du robot.
- **Remappings** : Les remappings des topics de transformation sont appliqués.

Le **`planner_server`** génère des plans de trajectoire pour le robot, en tenant compte de l'environnement et des obstacles.

#### **`recoveries_server`**
- **Package** : `nav2_recoveries`
- **Exécutable** : `recoveries_server`
- **Nom** : `recoveries_server`
- **Paramètres** : Les mêmes paramètres YAML que ceux utilisés pour **`controller_server`** et **`planner_server`**.
- **Remappings** : Remapping des topics de transformation.

Le **`recoveries_server`** gère les comportements de récupération, comme la rotation ou la translation, pour permettre au robot de se remettre d'un échec de planification.

#### **`bt_navigator`**
- **Package** : `nav2_bt_navigator`
- **Exécutable** : `bt_navigator`
- **Nom** : `bt_navigator`
- **Paramètres** : Le fichier de paramètres YAML et le fichier XML de l'arbre de comportements sont utilisés pour déterminer le comportement du robot.
- **Remappings** : Remapping des topics de transformation.

Le **`bt_navigator`** utilise un arbre de comportements pour gérer la navigation, la planification et les comportements de récupération du robot.

#### **`waypoint_follower`**
- **Package** : `nav2_waypoint_follower`
- **Exécutable** : `waypoint_follower`
- **Nom** : `waypoint_follower`
- **Paramètres** : Les paramètres de configuration pour suivre un chemin prédéfini.
- **Remappings** : Remapping des topics de transformation.

Le **`waypoint_follower`** permet au robot de suivre des waypoints ou des trajectoires pré-définis.

#### **`lifecycle_manager_navigation`**
- **Package** : `nav2_lifecycle_manager`
- **Exécutable** : `lifecycle_manager`
- **Nom** : `lifecycle_manager_navigation`
- **Paramètres** : Ce nœud gère le cycle de vie des nœuds de navigation, en utilisant des paramètres comme **`use_sim_time`**, **`autostart`**, et les noms des nœuds à gérer.
  - **`node_names`** : Liste des nœuds à gérer dans le cycle de vie.

Ce nœud assure le démarrage, la gestion et l'arrêt des nœuds dans la pile de navigation.

### **6. Lancement des nœuds**
La **`LaunchDescription`** retourne la liste des actions et des nœuds à lancer :
- **`SetEnvironmentVariable`** : Définit une variable d'environnement pour afficher les messages de log immédiatement sur la sortie standard.
- **`DeclareLaunchArgument`** : Déclare les arguments de lancement, qui peuvent être fournis lors du lancement pour configurer différents aspects du système (par exemple, la simulation, les paramètres, l'arbre de comportements, etc.).
- **Nœuds à lancer** :
  - **`controller_server`** : Lance le serveur de contrôle.
  - **`planner_server`** : Lance le serveur de planification.
  - **`recoveries_server`** : Lance le serveur de récupération.
  - **`bt_navigator`** : Lance le nœud de navigation basé sur un arbre de comportements.
  - **`waypoint_follower`** : Lance le nœud pour suivre les waypoints.
  - **`lifecycle_manager_navigation`** : Lance le gestionnaire de cycle de vie des nœuds de navigation.

---

## **Résumé du fichier `navigation_launch.py`**

Le fichier **navigation_launch.py** permet de configurer et de lancer les nœuds nécessaires pour la navigation du robot en utilisant la pile **Nav2**. Il configure et lance les nœuds suivants :
- **`controller_server`** : Contrôle les actionneurs du robot.
- **`planner_server`** : Planifie les trajectoires.
- **`recoveries_server`** : Gère les comportements de récupération.
- **`bt_navigator`** : Utilise un arbre de comportements pour naviguer.
- **`waypoint_follower`** : Fait suivre des waypoints au robot.
- **`lifecycle_manager_navigation`** : Gère le cycle de vie des nœuds de navigation.


