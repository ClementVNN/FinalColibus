# Fichier **online_async_launch.py** du projet **articubot_one**

Le fichier **online_async_launch.py** est utilisé pour lancer le nœud **`async_slam_toolbox_node`** du package **slam_toolbox**, qui est un outil de cartographie et de localisation en ligne pour les robots. Ce fichier permet de configurer des paramètres pour ce nœud en utilisant un fichier YAML et de gérer les paramètres de simulation. Il vérifie également si les paramètres nécessaires sont présents dans le fichier spécifié et, si ce n'est pas le cas, il utilise un fichier de paramètres par défaut.

---

## **Contenu du fichier `online_async_launch.py`**

### **1. Déclaration des importations**
- **`os`** : Utilisé pour interagir avec le système de fichiers, notamment pour obtenir les chemins des fichiers de paramètres.
- **`launch`** : Contient des classes et des fonctions pour créer des descriptions de lancement, déclarer des arguments de lancement et configurer les actions.
- **`launch_ros.actions.Node`** : Permet de définir et de lancer des nœuds ROS2 dans la description de lancement.
- **`ament_index_python.packages`** : Permet d'obtenir le chemin du répertoire de partage d'un package ROS2.
- **`nav2_common.launch.HasNodeParams`** : Vérifie si un fichier de paramètres contient les paramètres nécessaires pour un nœud donné (ici, **`slam_toolbox`**).
- **`launch.substitutions.PythonExpression`** : Permet de créer des expressions Python pour manipuler les variables de lancement de manière dynamique.

### **2. Déclaration des variables et configurations**
- **`use_sim_time`** : Déclare un paramètre de lancement pour utiliser l'horloge simulée (Gazebo) au lieu de l'horloge réelle.
- **`params_file`** : Fichier de paramètres fourni pour la configuration du nœud **slam_toolbox**.
- **`default_params_file`** : Fichier de paramètres par défaut qui est utilisé si le fichier de paramètres fourni ne contient pas les paramètres nécessaires pour **slam_toolbox**.
  - Ce fichier par défaut est situé dans le répertoire de configuration du package **articubot_one** sous le nom **`mapper_params_online_async.yaml`**.

### **3. Déclaration des arguments de lancement**
- **`declare_use_sim_time_argument`** : Permet de déclarer un argument de lancement **`use_sim_time`**, qui spécifie si l'horloge de simulation doit être utilisée.
- **`declare_params_file_cmd`** : Permet de déclarer un argument de lancement **`params_file`**, qui spécifie le chemin vers le fichier de paramètres utilisé pour configurer le nœud **slam_toolbox**.

### **4. Vérification des paramètres du nœud**
- **`has_node_params`** : Utilise **`HasNodeParams`** pour vérifier si le fichier de paramètres fourni contient les paramètres nécessaires pour le nœud **slam_toolbox**.
  - Si le fichier ne contient pas ces paramètres, le fichier de paramètres par défaut (**`mapper_params_online_async.yaml`**) sera utilisé à la place.
- **`actual_params_file`** : Utilise **`PythonExpression`** pour définir dynamiquement le fichier de paramètres réel qui sera utilisé en fonction de la présence ou non des paramètres dans le fichier fourni.

### **5. Configuration et lancement du nœud `slam_toolbox`**
- **`start_async_slam_toolbox_node`** : Lance le nœud **`async_slam_toolbox_node`** du package **slam_toolbox**.
  - Ce nœud est configuré avec les paramètres spécifiques (y compris **`use_sim_time`**) et le fichier de paramètres déterminé précédemment (soit le fichier fourni, soit le fichier par défaut).
  - Le nœud est configuré pour afficher sa sortie sur le terminal (`output='screen'`).

### **6. Description de lancement**
- **`LaunchDescription`** : Crée une description de lancement qui regroupe toutes les actions à exécuter lorsque le fichier de lancement est exécuté.
  - **`declare_use_sim_time_argument`** : Déclare l'argument de lancement pour l'horloge simulée.
  - **`declare_params_file_cmd`** : Déclare l'argument de lancement pour le fichier de paramètres.
  - **`log_param_change`** : Affiche un message dans le terminal si les paramètres nécessaires pour **slam_toolbox** ne sont pas présents dans le fichier de paramètres fourni, indiquant l'utilisation du fichier par défaut.
  - **`start_async_slam_toolbox_node`** : Lance le nœud **`async_slam_toolbox_node`** avec les paramètres appropriés.

---

## **Résumé du fichier `online_async_launch.py`**

Le fichier **online_async_launch.py** est utilisé pour lancer le nœud **`async_slam_toolbox_node`** avec les paramètres appropriés. Il vérifie si le fichier de paramètres fourni contient les paramètres nécessaires pour **slam_toolbox**. Si ce n'est pas le cas, un fichier de paramètres par défaut est utilisé à la place. Ce fichier de lancement configure également l'utilisation de l'horloge simulée (Gazebo) et affiche un message dans le terminal pour indiquer si le fichier de paramètres par défaut est utilisé.

Les principales étapes du lancement sont :
- Déclaration des arguments de lancement (**`use_sim_time`** et **`params_file`**).
- Vérification des paramètres dans le fichier de paramètres.
- Lancement du nœud **`async_slam_toolbox_node`** avec le fichier de paramètres approprié.

---

