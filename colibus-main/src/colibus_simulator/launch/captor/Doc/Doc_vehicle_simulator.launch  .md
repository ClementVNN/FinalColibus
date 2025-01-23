# Fichier **vehicle_simulator.launch.py**

Le fichier **`vehicle_simulator.launch.py`** est utilisé pour configurer et démarrer un nœud **vehicle_control_node** du paquet **`colibus_simulator`** dans un environnement ROS2. Ce nœud est responsable de la simulation et du contrôle du véhicule, notamment en permettant de commander et simuler le comportement du véhicule dans un environnement virtuel.

---

## **Contenu du fichier `vehicle_simulator.launch.py`**

### **1. Déclaration des importations**
- **`launch`** : Fournit les outils nécessaires pour définir et gérer des actions de lancement dans ROS2.
- **`launch_ros.actions.Node`** : Utilisé pour créer un nœud ROS2 à partir d'un paquet donné. Ici, ce mécanisme est utilisé pour démarrer le nœud **`vehicle_control_node`**.

### **2. Fonction `generate_launch_description`**
- **`LaunchDescription`** : La fonction **`generate_launch_description`** retourne une **`LaunchDescription`** qui définit les actions à exécuter lors du lancement.
  - **`Node`** : Cette action spécifie un nœud ROS2 à exécuter.
    - Le nœud **`vehicle_control_node`** appartient au paquet **`colibus_simulator`** et est exécuté pour gérer le contrôle du véhicule dans la simulation.
    - L'argument **`package='colibus_simulator'`** indique que le nœud appartient au paquet **`colibus_simulator`**.
    - L'argument **`executable='vehicle_control_node'`** spécifie que le nœud à exécuter est le **`vehicle_control_node`**, qui gère le contrôle du véhicule.
    - L'argument **`output='screen'`** spécifie que les sorties du nœud seront affichées sur le terminal lors de l'exécution.

### **3. Détails de l'exécution**
- Le nœud **`vehicle_control_node`** est exécuté en utilisant les configurations par défaut. Il n'y a pas de paramètres supplémentaires spécifiés dans ce fichier, ce qui signifie que le nœud démarrera avec les valeurs par défaut définies dans le paquet ou le code du nœud.
- Les sorties générées par ce nœud seront affichées dans le terminal pour un débogage ou une vérification en temps réel.

---

## **Résumé du fichier `vehicle_simulator.launch.py`**

Le fichier **`vehicle_simulator.launch.py`** permet de démarrer le nœud **`vehicle_control_node`** du paquet **`colibus_simulator`**. Ce nœud gère la simulation et le contrôle du véhicule dans l'environnement de simulation ROS2, en permettant de commander les mouvements du véhicule. La sortie du nœud est envoyée à l'écran, ce qui permet de visualiser toute information ou erreur générée par le nœud pendant son exécution.

---

