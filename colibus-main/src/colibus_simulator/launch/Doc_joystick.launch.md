# Fichier **joystick.launch.py** du projet **colibus_simulator**

Le fichier **joystick.launch.py** permet de configurer et de lancer les nœuds nécessaires à l'intégration d'un joystick pour contrôler le robot dans le projet **colibus_simulator**. Ce fichier configure deux nœuds principaux : un pour lire les entrées du joystick et un pour convertir ces entrées en commandes de mouvement (`Twist`).

---

## **Contenu du fichier `joystick.launch.py`**

### **1. Déclaration des importations**
- **`LaunchDescription`** : Permet de créer la structure de lancement du fichier.
- **`Node`** : Permet de définir les nœuds ROS à lancer.
- **`LaunchConfiguration`** : Utilisé pour récupérer les arguments de lancement, comme l'option pour utiliser le temps simulé (`use_sim_time`).
- **`DeclareLaunchArgument`** : Permet de déclarer un argument de lancement pour le paramètre `use_sim_time`.
- **`os` et `ament_index_python.packages`** : Utilisés pour gérer les chemins de fichiers dans le système de fichiers de ROS.

### **2. Paramétrage des nœuds**
Le fichier configure deux nœuds principaux qui sont lancés dans le cadre de l'utilisation du joystick.

#### **`joy_node`**
- **Package :** `joy`
- **Exécutable :** `joy_node`
- **Paramètres :**
  - **`joy_params`** : Fichier de configuration `joystick.yaml` qui contient les mappages des boutons et axes du joystick.
  - **`use_sim_time`** : Définit si le robot doit utiliser le temps simulé (`true`) ou non (`false`).

Ce nœud est responsable de lire les entrées du joystick et de les publier sur un topic ROS.

#### **`teleop_node`**
- **Package :** `teleop_twist_joy`
- **Exécutable :** `teleop_node`
- **Paramètres :**
  - **`joy_params`** : Le même fichier de configuration `joystick.yaml`.
  - **`use_sim_time`** : Paramètre pour le temps simulé.
- **Remapping :**
  - **`/cmd_vel` → `/cmd_vel_joy`** : Le topic de sortie pour les commandes de mouvement est remappé pour être accessible sous un autre nom (`/cmd_vel_joy`).

Le nœud `teleop_node` convertit les données du joystick en commandes de mouvement pour le robot sous forme de messages `Twist`.

---

### **3. Arguments de lancement**
Le fichier déclare un argument de lancement pour activer ou désactiver l'utilisation du temps simulé :

- **`use_sim_time`** : Par défaut, la valeur est définie sur `false`, mais elle peut être modifiée par l'utilisateur pour activer le temps simulé.

---

## **Résumé des nœuds lancés dans `joystick.launch.py`**

1. **`joy_node`** : Lit les entrées du joystick et publie les données sur un topic ROS.
2. **`teleop_node`** : Convertit les entrées du joystick en commandes de mouvement pour le robot sous forme de messages `Twist`.

Le fichier **joystick.launch.py** facilite ainsi l'intégration du contrôle du robot via un joystick, en permettant une configuration flexible pour l'utilisation dans une simulation ou dans un environnement réel.

---

