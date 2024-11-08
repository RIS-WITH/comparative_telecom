# Tests Réseau

Cette section contient les tests réseau effectués sur le projet. Actuellement, elle inclut des tests de latence sur le réseau local en utilisant les middlewares suivants :
- DDS de ROS2 (FastRTPS)
- FastDDS
- CycloneDDS
- Zenoh

Pour la communication avec le site web :
- ROSWebTools rosbridge_webserver - roslibjs et apache2
- RestAPI - zenoh, react, et react nextjs server

## Prérequis
- apache2
- ROS2 Iron
- Zenoh
- FastDDS
- CycloneDDS
- ROSWebTools

### Sur le Premier Système :
Tout d'abord, démarrez le serveur apache2 pour le site web :
```bash
sudo service apache2 start
```
Ensuite, naviguez vers le répertoire apache2 :
```bash
cd /var/www/html
```
Clonez le projet Comparative_Telecom ou juste le dossier IHM :
```bash
git clone
```
Créez un espace de travail ROS2 et ajoutez les trois nœuds :
- loki
- interfaces
- topic_service_lister
```bash
source /opt/ros/iron/setup.bash
mkdir -p ~/ros2_loki_ws/src
cd ~/ros2_loki_ws/src
cp -r /var/www/html/Comparatif_Telecom/IHM/tools/loki .
cp -r /var/www/html/Comparatif_Telecom/IHM/tools/interfaces .
cp -r /var/www/html/Comparatif_Telecom/IHM/tools/topic_service_lister .
```

Construisez les trois nœuds :
```bash
cd ~/ros2_loki_ws
colcon build
```

#### Configuration pour un Test DDS :
Ouvrez un terminal et choisissez un ID de domaine pour le test :
```bash
export ROS_DOMAIN_ID=42
```

Démarrez le rosbridge_server :
```bash
source /opt/ros/iron/setup.bash
source ~/ros2_loki_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml 
```
Ouvrez un autre terminal et démarrez le nœud loki :
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/iron/setup.bash
source ~/ros2_loki_ws/install/setup.bash
ros2 run loki loki
```

### Sur le Deuxième Système :

Créez un espace de travail ROS2 et ajoutez les deux nœuds :
- Yunobo
- Interfaces
```bash
source /opt/ros/iron/setup.bash
mkdir -p ~/ros2_yunobo_ws/src
cd ~/ros2_yunobo_ws/src
# copiez les deux nœuds
scp -r user@ip:/var/www/html/Comparatif_Telecom/IHM/tools/yunobo .
scp -r user@ip:/var/www/html/Comparatif_Telecom/IHM/tools/interfaces .
```

Construisez les deux nœuds :
```bash
cd ~/ros2_yunobo_ws
colcon build
```

#### Configuration pour un Test DDS :
Démarrez le nœud yunobo :
```bash
export ROS_DOMAIN_ID=42
source /opt/ros/iron/setup.bash
source ~/ros2_yunobo_ws/install/setup.bash
ros2 run yunobo yunobo_node --ros-args -p robot_ip:=<robot_ip>
```
- robot_ip : l'adresse IP du robot, donc le script sur le troisième système doit être en cours d'exécution avant de démarrer ce nœud.

### Sur le Troisième Système (Robot) :
Exécutez simplement l'un des scripts dans le dossier `IHM/tools/robot_scripts`.  
Exemple pour exécuter le script python sur pepper :
```bash
python3 robot_server.py
```

### Sur la Machine de Lancement de Test :
Ouvrez un navigateur et allez sur le site web où Apache2 est en cours d'exécution :
```bash
http://localhost/Comparatif_Telecom/IHM/test1-RosWebTools/
```

Démarrez les tests de latence en cliquant sur le bouton `Start Test`. Après la fin du test, la couleur du bouton reviendra à sa couleur initiale. Cliquez sur le bouton `Download` pour télécharger le fichier de résultat.


TODO : comment changer entre les différents middlewares et les différents tests.