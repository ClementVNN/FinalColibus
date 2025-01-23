# Structure du projet **colibus_simulator**

Ce document décrit la structure des dossiers et fichiers du projet **colibus_simulator**, le projet ROS 2 avec simulation Gazebo, support graphique, et messages personnalisés.

---

## **1. Dossiers principaux**

### **`config`**
Ce dossier contient des fichiers de configuration pour le simulateur ou les nœuds ROS 2. 
- Ces fichiers sont souvent utilisés pour définir des paramètres spécifiques au comportement des nœuds ou du simulateur.
- Les fichiers sont au format YAML ou JSON, et permettent de configurer des éléments comme les capteurs, les robots, ou les comportements spécifiques de la simulation.

---

### **`icars_msgs/msg`**
Ce dossier contient des définitions de messages personnalisés pour le projet ROS de véhicule réel (pas de la simulation). 
- Dans un projet ROS, les messages sont des structures de données envoyées entre différents nœuds via la communication de type publisher/subscriber.
- Les fichiers `.msg` sont utilisés pour définir les messages échangés, en spécifiant les types de données et leur disposition.

---

### **`launch`**
Ce dossier contient des fichiers de lancement ROS 2.
- Ces fichiers permettent d'orchestrer le démarrage des différents éléments du système, comme le simulateur, les nœuds ROS, et le chargement des modèles URDF.
- Un fichier de lancement peut configurer des environnements, charger des modèles URDF de robots, configurer des capteurs, etc.

---

### **`meshes`**
Ce dossier contient des modèles 3D au format **mesh** pour les objets simulés dans le projet.
- Ces fichiers représentent visuellement des objets comme des robots, des pièces, ou des environnements.
- Les fichiers de ce dossier sont utilisés pour afficher les objets dans le simulateur.

---

### **`nodes`**
Ce dossier contient les scripts définissant les nœuds ROS 2, écrits en Python.
- Un nœud ROS est une unité logique du système qui exécute une tâche spécifique, comme la lecture d'un capteur, le traitement d'une donnée, ou le contrôle d'un robot.
- Chaque fichier dans ce dossier contient la logique pour un nœud, et peut inclure des tâches comme la publication de messages, l'abonnement à des topics, ou la gestion des capteurs et des actionneurs.

---

### **`resource`**
Ce dossier contient des fichiers supplémentaires qui sont nécessaires au projet, mais qui ne rentrent pas directement dans les catégories précédentes.

---

### **`ros_zoe_msgs`**
Ce dossier est similaire à `icars_msgs`, mais contient des messages ou services ROS personnalisés qui sont spécifiques au véhicule réel reprenant le projet développé au préalable sur la Renault Zoe.

---

### **`test`**
Ce dossier contient des scripts de test utilisés pour valider les fonctionnalités du projet.
- Ces tests peuvent être des tests unitaires pour valider des composants spécifiques du système, ou des tests d'intégration pour vérifier que l'ensemble du système fonctionne comme prévu.
- **Exemple** :
  - `test_simulation.py` : Un fichier Python qui pourrait vérifier si les capteurs simulés génèrent les bonnes données ou si les nœuds interagissent correctement.

---

### **`urdf`**
Ce dossier contient des fichiers URDF (**Unified Robot Description Format**) qui décrivent la structure physique du robot.
- Les fichiers URDF sont utilisés pour définir le robot dans ROS, incluant ses liens (parties du robot), ses joints (connexions entre les liens), et ses capteurs.
- Ces fichiers sont cruciaux pour la simulation et le contrôle du robot.

---

### **`urdf_old`**
Ce dossier contient une version ancienne des fichiers URDF, pour référence.

---

### **`worlds`**
Ce dossier contient des fichiers de description de monde pour la simulation.
- Ces fichiers définissent l'environnement dans lequel le robot évolue (par exemple, la ville, la rue, ou d'autres scénarios simulés).

---

## **2. Fichiers principaux**

### **`CMakeLists.txt`**
Le fichier `CMakeLists.txt` est utilisé par CMake pour configurer la compilation et la construction du package ROS 2.
- Il définit les dépendances du package, les fichiers à compiler, et les cibles à générer pour le projet.

---

### **`__init__.py`**
Le fichier `__init__.py` est un fichier Python qui indique que le répertoire est un package Python.
- Ce fichier est nécessaire pour que Python reconnaisse ce répertoire comme un module importable.

---

### **`package.xml`**
Le fichier `package.xml` est un fichier de configuration pour ROS 2, décrivant le package.
- Il contient des métadonnées sur le package, comme le nom, la version, et les dépendances nécessaires à son fonctionnement.
