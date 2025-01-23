# Fichier **`config.py`**

Le fichier **`config.py`** contient diverses variables et paramètres utilisés pour configurer et ajuster le comportement du système, notamment pour la détection de voies, la gestion des caméras et la simulation d'une navigation autonome. Ce fichier permet de contrôler différents aspects du système, tels que le débogage, la détection des voies, la simulation de la navigation par satellite et le contrôle des vidéos d'entrée et de sortie.

---

## **Contenu du fichier `config.py`**

### **1. Déclaration des importations**
- **`os`** : Utilisé pour interagir avec le système de fichiers, notamment pour obtenir les chemins absolus des fichiers vidéo.
- **`cv2`** : Utilisé pour manipuler des vidéos et des images avec OpenCV, notamment pour écrire des vidéos de sortie et effectuer des transformations d'images.

### **2. Variables de Contrôle**
Les variables suivantes contrôlent l'activation ou la désactivation de certaines fonctionnalités du système :

- **`detect`** : Si défini sur `1`, active la détection de voie.
- **`Testing`** : Si défini sur `True`, permet de voir ce que le robot "voit" via la caméra.
- **`Profiling`** : Si défini sur `True`, active la profilisation du code pour l'optimisation des performances.
- **`write`** : Si défini sur `True`, permet d'écrire les vidéos d'entrée et de sortie.
- **`In_write` et `Out_write`** : Contrôlent respectivement l'écriture des vidéos d'entrée et de sortie.
- **`debugging`** : Si défini sur `True`, permet d'activer les messages de débogage dans le code.
- **`debugging_Lane`** : Active les messages de débogage pour la détection de voie.
- **`debugging_L_ColorSeg`**, **`debugging_L_Est`**, **`debugging_L_Cleaning`**, **`debugging_L_LaneInfoExtraction`** : Ces variables activent respectivement le débogage pour chaque étape du processus de détection de voie.
- **`debugging_Signs`** et **`debugging_TrafficLights`** : Activent le débogage pour la détection des panneaux et des feux de circulation.
- **`debugging_TL_Config`** : Active le débogage pour la configuration des feux de circulation.
- **`enable_SatNav`** : Si défini sur `True`, active la navigation par satellite.
- **`animate_steering`** : Si défini sur `True`, active l'animation de la direction du robot.
- **`angle_orig` et `angle`** : Contiennent respectivement l'angle d'origine et l'angle de direction lissé pour l'animation de direction.
- **`engines_on`** : Si défini sur `True`, les moteurs du robot sont activés.
- **`clr_seg_dbg_created`** : Cette variable empêche la création de barres de débogage de segmentation de couleur plusieurs fois.
- **`Detect_lane_N_Draw`** : Si défini sur `True`, la détection et l'affichage des voies sont activés.
- **`Training_CNN`** : Si défini sur `True`, active l'entraînement d'un réseau de neurones convolutionnel pour la détection de voies.
  
### **3. Paramètres de vidéo et de sortie**
- **`vid_path`** : Spécifie le chemin du fichier vidéo à utiliser pour la détection de voies.
- **`loopCount`** : Un compteur utilisé pour contrôler la boucle de traitement vidéo.
- **`Resized_width` et `Resized_height`** : Définissent la largeur et la hauteur des vidéos redimensionnées pour la détection de voies (320x240).
- **`in_q` et `out`** : Spécifient les objets **`cv2.VideoWriter`** pour écrire les vidéos d'entrée et de sortie respectivement.

### **4. Paramètres pour la détection de voies**
Ces paramètres définissent la manière dont les images sont traitées et analysées pour la détection de voies :
- **`Ref_imgWidth` et `Ref_imgHeight`** : La largeur et la hauteur de l'image de référence utilisée pour le calcul de la détection de voie (1920x1080).
- **`Frame_pixels`** : Le nombre total de pixels dans l'image de référence.
- **`Resize_Framepixels`** : Le nombre total de pixels dans l'image redimensionnée.
- **`Lane_Extraction_minArea_per`** : Définit la zone minimale relative d'une voie qui doit être détectée.
- **`minArea_resized`** : Calcul de la zone minimale pour les images redimensionnées.
- **`BWContourOpen_speed_MaxDist_per`** : Paramètre lié à la distance maximale de la détection de contours pour les images en noir et blanc.
- **`MaxDist_resized`** : La distance maximale calculée pour les images redimensionnées.
- **`CropHeight` et `CropHeight_resized`** : Définit la hauteur de découpe des images pour la détection de voies.

### **5. Contrôle du temps d'attente**
- **`waitTime`** : Contrôle le temps d'attente entre les images traitées en mode débogage.

---

## **Résumé du fichier `config.py`**

Le fichier **`config.py`** contient des variables de configuration essentielles pour contrôler différents aspects du système de détection de voies et de navigation autonome. Il permet de personnaliser des paramètres tels que la détection des voies, le débogage, l'écriture des vidéos, ainsi que la gestion de la navigation par satellite et de l'animation de la direction. Les paramètres de taille d'image et de détection sont également définis pour optimiser le traitement des images et garantir une détection de voie précise.



---
