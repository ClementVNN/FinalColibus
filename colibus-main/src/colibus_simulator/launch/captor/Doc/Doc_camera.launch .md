# Fichier **camera.launch.py**

Le fichier **camera.launch.py** permet de configurer et de lancer un nœud **v4l2_camera_node** dans un environnement ROS2. Ce nœud est utilisé pour interagir avec une caméra vidéo (généralement via un périphérique vidéo V4L2 sous Linux). Le fichier de lancement configure les paramètres du flux vidéo, tels que la taille de l'image et le taux de rafraîchissement.

---

## **Contenu du fichier `camera.launch.py`**

### **1. Déclaration des importations**
- **`os`** : Permet d'interagir avec le système de fichiers. Bien que cet import ne soit pas utilisé directement dans ce fichier, il peut être utilisé pour étendre la configuration (par exemple, pour obtenir des chemins relatifs ou absolus).
- **`launch`** : Fournit des classes et des fonctions pour définir et gérer les descriptions de lancement dans ROS2.
- **`launch_ros.actions.Node`** : Permet de créer et de gérer les nœuds ROS2 dans une description de lancement.

### **2. Fonction `generate_launch_description`**
- **`LaunchDescription`** : La fonction **`generate_launch_description()`** crée une **`LaunchDescription`**, qui est un conteneur pour toutes les actions que ROS2 doit exécuter lorsqu'on lance ce fichier.
  - **`Node`** : Dans cette description de lancement, un nœud est défini pour le paquet **`v4l2_camera`**. Ce nœud, **`v4l2_camera_node`**, est responsable de l'acquisition d'images depuis une caméra compatible V4L2 (Video for Linux 2).
  - Le **nœud** reçoit plusieurs paramètres sous forme de dictionnaire :
    - **`image_size`** : Définit la taille de l'image capturée par la caméra. Ici, la taille est fixée à **640x480** pixels.
    - **`time_per_frame`** : Spécifie le temps nécessaire pour capturer chaque image en termes de durées. La valeur est donnée sous forme de liste **[1, 6]**.
    - **`camera_frame_id`** : Définit l'identifiant du cadre de la caméra, ici, **`camera_link_optical`**, ce qui signifie que le cadre de référence pour l'optique de la caméra est **camera_link_optical**.

### **3. Détails de l'exécution**
- Le nœud **`v4l2_camera_node`** est exécuté avec les paramètres définis et le flux vidéo est envoyé à la sortie **`screen`** pour affichage dans le terminal lors de l'exécution du lancement.
- Le **namespace** du nœud est défini sur **`camera`**, ce qui permet de le distinguer des autres nœuds potentiels et de l'organiser dans un espace de noms séparé dans ROS2.

---

## **Résumé du fichier `camera.launch.py`**

Le fichier **`camera.launch.py`** est utilisé pour démarrer le nœud **`v4l2_camera_node`** dans un espace de noms **`camera`**. Ce nœud est responsable de la capture d'images à partir d'une caméra compatible **V4L2**. Les paramètres de lancement incluent la taille de l'image, le temps par image et l'identifiant du cadre de la caméra. Le fichier configure ainsi les paramètres vidéo essentiels pour que le flux de caméra fonctionne correctement dans un environnement ROS2.


---

