# Fichier **`ekf.yaml`**

Le fichier **`ekf.yaml`** est une configuration pour le filtre de Kalman étendu (EKF) dans un système ROS2. Ce filtre est utilisé pour fusionner différentes sources de données provenant de capteurs, telles que le GPS et les odométries, afin de fournir une estimation plus précise de l'état du robot, notamment sa position et sa vitesse.

---

## **Contenu du fichier `ekf.yaml`**

### **1. Paramètres généraux**
- **`frequency`** : La fréquence à laquelle le filtre EKF est exécuté (en Hz). Ici, elle est définie sur **30 Hz**.
- **`sensor_timeout`** : Le délai d'attente (en secondes) avant de considérer que les données d'un capteur sont périmées. Ici, il est défini sur **0.1 seconde**.
- **`two_d_mode`** : Définit si le mode 2D est activé. Lorsque défini sur **false**, le filtre utilise les données en 3D.
- **`transform_time_offset`** : Décalage temporel appliqué lors de la transformation entre les cadres de référence (en secondes). Ici, il est défini sur **0.0 seconde**.
- **`transform_timeout`** : Le délai d'attente avant d'échouer la transformation entre les cadres de référence (en secondes). Défini sur **0.0 seconde**.
- **`print_diagnostics`** : Si défini sur **true**, le système imprime des diagnostics dans la console.
- **`debug`** : Si défini sur **false**, le mode débogage est désactivé.

### **2. Paramètres des cadres de référence**
- **`map_frame`** : Le cadre de référence de la carte, ici défini sur **`map`**.
- **`odom_frame`** : Le cadre de référence de l'odométrie, ici défini sur **`odom`**.
- **`base_link_frame`** : Le cadre de référence de la base du robot, ici défini sur **`base_link`**.
- **`world_frame`** : Le cadre de référence global, ici défini sur **`odom`**.

### **3. Configuration du GPS**
- **`gps0`** : Le topic sur lequel les données GPS sont reçues. Ici, il est défini sur **`/gps/fix`**.
- **`gps0_pose_frame`** : Le cadre de référence pour les poses GPS, ici défini sur **`gps_link`**.
- **`gps0_position_covariance`** : La covariance de la position GPS sous forme de liste. Ici, elle est définie comme **[1, 1, 1]**, indiquant une covariance diagonale.
- **`gps0_position_covariance_type`** : Le type de covariance pour la position GPS. Ici, il est défini sur **`diagonal`**, ce qui signifie que la covariance est représentée par une matrice diagonale.

### **4. Configuration de l'odométrie SLAM**
- **`odom0`** : Le topic où les données d'odométrie SLAM sont publiées. Ici, il est défini sur **`/slam/odometry`**.
- **`odom0_frame`** : Le cadre de référence des données d'odométrie SLAM, ici défini sur **`slam_link`**.
- **`odom0_pose_rejection_threshold`** : Le seuil de rejet de la pose d'odométrie SLAM. Si l'écart entre les valeurs mesurées et estimées dépasse ce seuil, la pose est rejetée. Ici, il est défini sur **5**.
- **`odom0_twist_rejection_threshold`** : Le seuil de rejet de la vitesse angulaire et linéaire d'odométrie SLAM. Si l'écart entre les vitesses mesurées et estimées dépasse ce seuil, la vitesse est rejetée. Ici, il est défini sur **1**.

---

## **Résumé du fichier `ekf.yaml`**

Le fichier **`ekf.yaml`** configure un filtre de Kalman étendu (EKF) pour fusionner les données de différents capteurs et obtenir une estimation précise de la position et de la vitesse du robot. Il inclut des paramètres pour la fréquence de calcul, les délais d'attente des capteurs, et des réglages spécifiques pour les données GPS et d'odométrie SLAM. Les données GPS et SLAM sont fusionnées pour offrir une estimation plus robuste de l'état du robot dans un environnement dynamique.

---
