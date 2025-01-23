# Fichier **`camera_front.py`**

Le fichier **`camera_front.py`** est un fichier ROS2 permettant de capturer et d'enregistrer des images en provenance d'une caméra. Ce nœud ROS2 s'abonne au flux d'images d'une caméra située à l'avant du véhicule, traite les images reçues, puis les écrit dans un fichier vidéo. Ce processus est effectué en temps réel, et les images sont également affichées sur un écran.

---

## **Contenu du fichier `camera_front.py`**

### **1. Importation des modules**
- **`rclpy`** : Le module nécessaire pour créer des nœuds dans ROS2.
- **`cv2`** : Utilisé pour le traitement d'images, ici pour afficher et enregistrer les vidéos.
- **`CvBridge`** : Permet de convertir les messages ROS d'images en format OpenCV (utilisé pour le traitement d'images).
- **`sensor_msgs.msg.Image`** : Le type de message utilisé pour recevoir des images via ROS.

### **2. Classe `Video_get`**
- **`Video_get`** : Une classe héritée de **`Node`** qui crée un nœud ROS2 pour recevoir les images et les traiter. Ce nœud est conçu pour s'abonner à un topic de caméra (ici, **`/upper_camera/image_raw`**) et pour enregistrer ces images dans un fichier vidéo.
  - **`self.subscriber`** : Un abonnement à un topic ROS2 qui reçoit les images envoyées par la caméra frontale du véhicule. Lorsque de nouvelles images sont reçues, elles sont traitées par la fonction **`process_data`**.
  - **`self.out`** : Un objet **`cv2.VideoWriter`** utilisé pour enregistrer les images sous forme de vidéo dans un fichier **`output.avi`** situé dans le répertoire **`/home/luqman/`**.
  - **`self.bridge`** : Un objet **`CvBridge`** utilisé pour convertir les messages d'images ROS en images OpenCV pour un traitement facile.
  
### **3. Fonction `process_data`**
- **`process_data`** : Cette fonction est appelée chaque fois qu'une nouvelle image est reçue.
  - **`frame = self.bridge.imgmsg_to_cv2(data, 'bgr8')`** : Cette ligne convertit les données d'image reçues du message ROS en une image OpenCV au format **BGR**.
  - **`self.out.write(frame)`** : L'image est ensuite écrite dans le fichier vidéo.
  - **`cv2.imshow("output", frame)`** : L'image est affichée dans une fenêtre nommée "output", permettant de visualiser ce qui est capturé en temps réel.
  - **`cv2.waitKey(1)`** : Permet de maintenir l'affichage de l'image pendant une courte période, jusqu'à ce que l'utilisateur interagisse (en appuyant sur une touche pour arrêter le processus).

### **4. Fonction `main`**
- **`main`** : Fonction principale qui initialise ROS2 et lance le nœud **`Video_get`**. Une fois le nœud en marche, il attend les messages d'images de la caméra.
  - **`rclpy.init(args=args)`** : Initialise l'environnement ROS2.
  - **`image_subscriber = Video_get()`** : Crée une instance de la classe **`Video_get`**.
  - **`rclpy.spin(image_subscriber)`** : Lance l'exécution continue du nœud, en attendant les messages entrants.
  - **`rclpy.shutdown()`** : Arrête le nœud lorsque l'exécution est terminée.

---

## **Résumé du fichier `camera_front.py`**

Le fichier **`camera_front.py`** crée un nœud ROS2 qui s'abonne à un topic d'images provenant d'une caméra frontale (nommé **`/upper_camera/image_raw`**). Ce nœud reçoit les images, les convertit en format OpenCV, puis les enregistre dans un fichier vidéo tout en les affichant en temps réel à l'écran. Ce processus permet de créer un enregistrement vidéo continu des images capturées par la caméra frontale.

---

