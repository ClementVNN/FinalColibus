# Fichier **localization_launch.py** du projet **articubot_one**

Le fichier **localization_launch.py** est responsable du lancement des nœuds nécessaires à la localisation du robot dans un environnement donné. Il configure les paramètres pour les serveurs de cartes et les algorithmes de localisation, tels que **`map_server`** et **`amcl`**. Ce fichier inclut également la gestion des paramètres de simulation et de l'automatisation du démarrage de la pile de navigation.

---

## **Contenu du fichier `localization_launch.py`**

### **1. Déclaration des importations**
- **`os`** : Permet d'interagir avec le système de fichiers, principalement pour accéder aux chemins des fichiers de configuration.
- **`ament_index_python.packages`** : Utilisé pour obtenir le chemin du répertoire de partage du package ROS2.
- **`launch`** : Contient des classes et fonctions permettant de créer des descriptions de lancement, de déclarer des arguments de lancement et de configurer les actions.
- **`launch_ros.actions.Node`** : Permet de définir et de lancer des nœuds ROS2 dans la description de lancement.
- **`nav2_common.launch.RewrittenYaml`** : Permet de manipuler et de réécrire des fichiers YAML avec des substitutions dans les paramètres de lancement.

### **2. Déclaration des variables et configurations**
- **`bringup_dir`** : Répertoire où se trouvent les fichiers de configuration et les cartes pour le robot, obtenu à partir du package **articubot_one**.
- **`namespace`** : Un paramètre de lancement optionnel pour définir un espace de noms pour les nœuds.
- **`map_yaml_file`** : Le fichier YAML contenant la carte à charger pour la localisation.
- **`use_sim_time`** : Détermine si le robot utilise le temps simulé (via Gazebo) ou le temps réel.
- **`autostart`** : Spécifie si la pile de navigation (`nav2`) doit démarrer automatiquement.
- **`params_file`** : Fichier YAML contenant les paramètres ROS2 à utiliser pour la configuration de la navigation.
- **`lifecycle_nodes`** : Liste des nœuds qui seront gérés par le gestionnaire de cycle de vie. Dans ce cas, ce sont les nœuds **`map_server`** et **`amcl`**.

### **3. Création des remappings**
Les remappings sont utilisés pour rediriger certains topics vers d'autres noms. Par exemple :
- **`/tf` → `tf`** : Remapping des topics de transformation.
- **`/tf_static` → `tf_static`** : Remapping pour les transformations statiques.

### **4. Configuration des paramètres via YAML**
Le fichier YAML contenant les paramètres pour la localisation et la carte est configuré avec des substitutions :
- **`use_sim_time`** et **`yaml_filename`** sont des substitutions pour indiquer le fichier de carte et la nécessité d'utiliser le temps simulé.
- **`RewrittenYaml`** : Permet de réécrire le fichier de paramètres avec les substitutions et de charger les configurations de manière dynamique.

### **5. Définition des nœuds ROS2 à lancer**

#### **`map_server`**
- **Package** : `nav2_map_server`
- **Exécutable** : `map_server`
- **Nom** : `map_server`
- **Paramètres** : Le fichier de paramètres YAML réécrit avec les substitutions pour la carte et le temps simulé.
- **Remappings** : Les topics `/tf` et `/tf_static` sont remappés.
  
Ce nœud charge la carte et la met à disposition pour les autres nœuds de navigation.

#### **`amcl` (Adaptive Monte Carlo Localization)**
- **Package** : `nav2_amcl`
- **Exécutable** : `amcl`
- **Nom** : `amcl`
- **Paramètres** : Les mêmes paramètres réécrits que ceux pour le **`map_server`**, afin d'utiliser la carte et le temps simulé.
- **Remappings** : Les remappings pour les topics de transformation sont appliqués ici aussi.

Le nœud **`amcl`** utilise l'algorithme de localisation Monte Carlo pour localiser le robot en fonction de la carte et des capteurs.

#### **`lifecycle_manager_localization`**
- **Package** : `nav2_lifecycle_manager`
- **Exécutable** : `lifecycle_manager`
- **Nom** : `lifecycle_manager_localization`
- **Paramètres** : Ce nœud gère l'état du cycle de vie des nœuds de localisation et de carte (comme `map_server` et `amcl`), en fonction des paramètres `use_sim_time` et `autostart`.
  - **`node_names`** : Liste des nœuds gérés par ce gestionnaire de cycle de vie (ici, `map_server` et `amcl`).

Ce nœud gère le démarrage et la gestion des nœuds de localisation pendant l'exécution du système.

### **6. Lancement des nœuds**
La **`LaunchDescription`** retourne la liste des actions et des nœuds à lancer :
- **`SetEnvironmentVariable`** : Définit une variable d'environnement pour afficher les messages de log immédiatement sur la sortie standard.
- **`DeclareLaunchArgument`** : Déclare les arguments de lancement, qui peuvent être fournis lors du lancement pour configurer différents aspects du système (par exemple, la carte, les paramètres, etc.).
- **Nœuds à lancer** :
  - **`map_server`** : Lance le serveur de carte avec la configuration de la carte et des remappings.
  - **`amcl`** : Lance le nœud de localisation **AMCL** avec la configuration et les remappings.
  - **`lifecycle_manager_localization`** : Lance le gestionnaire de cycle de vie pour gérer les nœuds **`map_server`** et **`amcl`**.

---

## **Résumé du fichier `localization_launch.py`**

Le fichier **localization_launch.py** permet de configurer et de lancer les nœuds nécessaires à la localisation du robot dans un environnement, en utilisant la carte et les algorithmes de localisation. Il configure et lance les nœuds suivants :
- **`map_server`** : Chargement et publication de la carte.
- **`amcl`** : Localisation du robot en utilisant l'algorithme Monte Carlo adapté.
- **`lifecycle_manager_localization`** : Gestion du cycle de vie des nœuds de localisation pour assurer un démarrage et une gestion appropriés.

---
