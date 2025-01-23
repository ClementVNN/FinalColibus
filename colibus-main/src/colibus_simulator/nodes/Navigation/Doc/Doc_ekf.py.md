# Fichier **`ekf.py`**

Le fichier **`ekf.py`** est responsable de la configuration et de l'exécution d'un filtre de Kalman étendu (EKF) dans un environnement ROS2. Ce filtre est utilisé pour fusionner différentes sources de données de capteurs, comme le GPS et l'odométrie, afin d'estimer la position et la vitesse du robot de manière plus précise.

---

## **Contenu du fichier `ekf.py`**

### **1. Paramètres généraux de la configuration**

- **`frequency: 30`** : La fréquence de mise à jour du filtre EKF est définie à 30 Hz, ce qui signifie que les estimations seront recalculées 30 fois par seconde.
- **`sensor_timeout: 0.1`** : Définit un délai d'attente pour chaque capteur à 0.1 secondes. Si le capteur ne transmet pas de données dans ce délai, il sera ignoré.
- **`two_d_mode: false`** : Lorsque ce paramètre est défini sur **`false`**, le filtre EKF fonctionne en 3D. Si défini sur **`true`**, il fonctionne en mode 2D, ce qui ignore la composante **`z`** pour la position.
- **`transform_time_offset: 0.0`** : Cette valeur ajuste le temps des transformations (en secondes) entre les différents cadres de référence. Un offset de 0 signifie qu'il n'y a pas de décalage.
- **`transform_timeout: 0.0`** : Le délai d'attente pour la transformation entre les systèmes de coordonnées, défini à 0 signifie qu'il n'y a pas de délai.
- **`print_diagnostics: true`** : Permet d'afficher les diagnostics du filtre EKF.
- **`debug: false`** : Permet de désactiver ou d'activer les messages de débogage supplémentaires. Ici, il est désactivé.

### **2. Paramètres liés aux cadres de référence**

- **`map_frame: map`** : Le cadre de référence de la carte est défini comme **`map`**.
- **`odom_frame: odom`** : Le cadre de référence de l'odométrie est défini comme **`odom`**.
- **`base_link_frame: base_link`** : Le cadre de référence pour la base du robot est défini comme **`base_link`**.
- **`world_frame: odom`** : Le cadre de référence global du monde est défini comme **`odom`**.

### **3. Configuration du GPS**

- **`gps0: /gps/fix`** : Le topic **`/gps/fix`** est utilisé pour recevoir les données GPS du robot.
- **`gps0_pose_frame: gps_link`** : Le cadre de référence associé aux données GPS est défini comme **`gps_link`**.
- **`gps0_position_covariance: [1, 1, 1]`** : Définit la covariance de position du GPS dans les axes **`x`**, **`y`** et **`z`**. Ici, la covariance est égale sur les trois axes.
- **`gps0_position_covariance_type: diagonal`** : Définit le type de covariance comme étant **diagonal**, ce qui signifie que les variances sont indépendantes et les covariances croisées sont nulles.

### **4. Configuration de l'odométrie SLAM**

- **`odom0: /slam/odometry`** : Le topic **`/slam/odometry`** est utilisé pour recevoir les données d'odométrie provenant de SLAM.
- **`odom0_frame: slam_link`** : Le cadre de référence pour les données d'odométrie SLAM est défini comme **`slam_link`**.
- **`odom0_pose_rejection_threshold: 5`** : Le seuil de rejet pour la pose (position) dans les données d'odométrie SLAM est défini à 5. Si une pose dépasse ce seuil, elle sera rejetée.
- **`odom0_twist_rejection_threshold: 1`** : Le seuil de rejet pour la vitesse dans les données d'odométrie SLAM est défini à 1. Si une vitesse dépasse ce seuil, elle sera rejetée.

---

## **Résumé du fichier `ekf.py`**

Le fichier **`ekf.py`** configure un filtre de Kalman étendu (EKF) pour fusionner les données provenant de différents capteurs, tels que le GPS et l'odométrie SLAM, afin d'améliorer l'estimation de la position et de la vitesse du robot. Le fichier contient des paramètres pour ajuster la fréquence de mise à jour, les seuils de rejet des capteurs, ainsi que les cadres de référence pour chaque source de données.

---

