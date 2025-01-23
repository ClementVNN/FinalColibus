# Fichier **`speed_accel.py`**

Le fichier **`speed_accel.py`** est un fichier ROS2 qui définit un **nœud** pour le contrôle du véhicule. Ce nœud est responsable de la gestion des commandes de vitesse et d'accélération du véhicule, en prenant en compte des données externes reçues via un abonnement à un topic, puis en calculant l'angle de direction nécessaire à partir des commandes d'accélération et de freinage.

---

## **Contenu du fichier `speed_accel.py`**

### **1. Importation des modules**
- **`rclpy`** : Le module de base pour créer des nœuds dans ROS2.
- **`std_msgs.msg.Bool`** : Utilisé pour échanger des messages de type booléen.
- **Messages personnalisés** : Des messages spécifiques au contrôle du véhicule sont importés à partir de **`your_package_name.msg`**. Parmi ces messages, on trouve :
  - **`CarStatus`** : Pour la gestion du statut du véhicule.
  - **`ClimaStatus`** : Pour le contrôle de la climatisation du véhicule.
  - **`CarId`** : Pour identifier un véhicule unique.
  - **`LogErr`** : Pour l'enregistrement des erreurs du système.
  - **`PID`** : Pour les paramètres PID du contrôle.
  - **`SteeringData`** : Pour l'angle de direction calculé.
  - **`SpeedAccelData`** : Pour les données de vitesse et d'accélération.
  - **`BlkLights`** : Pour contrôler les feux du véhicule.
  - **`Beep`** : Pour envoyer un signal sonore.
  - **`LCDText`** : Pour afficher des informations sur un écran LCD.
  
- **Services personnalisés** : Des services sont définis pour interagir avec le véhicule, tels que :
  - **`SetControlMode`** : Pour définir le mode de contrôle du véhicule.
  - **`GetControlMode`** : Pour récupérer le mode de contrôle actuel.
  - **`PIDParams`** : Pour obtenir ou définir les paramètres du PID.
  - **`GetBlStatus`** : Pour obtenir le statut des feux.
  - **`GetRunMode`** : Pour récupérer le mode d'exécution du véhicule.

### **2. Classe `VehicleControlNode`**
- **`VehicleControlNode`** : Un nœud ROS2 qui gère les commandes du véhicule. Il est initialisé avec le nom **`vehicle_control_node`** et crée un abonnement à un topic de données de vitesse et d'accélération.
  - **`self.speed_accel_data_sub`** : Un abonnement aux messages **`SpeedAccelData`** sur le topic **`/VEHICLE/control/speed_accel_data`**.
  - Ce nœud est configuré pour recevoir des données de vitesse et d'accélération qui influencent directement l'angle de direction du véhicule.
  
### **3. Fonction `handle_speed_accel_data`**
- **`handle_speed_accel_data`** : Cette fonction est appelée lorsque de nouvelles données de vitesse et d'accélération sont reçues. 
  - **Données reçues** :
    - **`acceleration`** : La commande d'accélération du véhicule.
    - **`braking`** : La commande de freinage du véhicule.
  - **Calcul de l'angle de direction** : L'angle de direction est calculé en fonction de la différence entre l'accélération et le freinage, multipliée par un gain proportionnel **`K`**.
  - **Publication des données de direction** : Un message **`SteeringData`** est créé avec l'angle de direction calculé et est publié sur un topic, contrôlant ainsi la direction du véhicule.

---

## **Résumé du fichier `speed_accel.py`**

Le fichier **`speed_accel.py`** gère le contrôle du véhicule dans un environnement ROS2. Il abonne le nœud à un topic pour recevoir des données de vitesse et d'accélération. Ces données sont utilisées pour calculer l'angle de direction du véhicule en fonction des commandes d'accélération et de freinage. Ensuite, un message **`SteeringData`** est généré et publié, permettant ainsi de contrôler la direction du véhicule en temps réel. Le gain proportionnel **`K`** permet d'ajuster la sensibilité du calcul de l'angle de direction.

---
