
# Test De Recrutement iFollow

Dans ce fichier Readme, nous avons mis les réponses au exercices qui permettent au recruteur d'évaluer es capacités du candidat à intégrer et utiliser
différentes technologies logicielles au sein du framework ROS.


## Requirements
Ubuntu 20.04.4 LTS

ROS Noetic Ninjemys 
## 1. Mise en place de l’environnement de test :
- La visualisation du robot et d’une map dans Rviz :

Install TurtleBot3 Packages:

```bash
sudo apt-get install ros-noetic-turtlebot3-msgs
sudo apt-get install ros-noetic-dynamixel-sdk
sudo apt-get install ros-noetic-turtlebot3
sudo apt install ros-noetic-slam-gmapping
```

Set TurtleBot3 Model Name :
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
```

Run Gazebo, Teleoperation Node and SLAM Node
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch 
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
```

Save the map
```bash
rosrun map_server map_saver -f ~/map
```

- Stack de localisation et de navigation permettant au robot de se localiser et de se déplacer dans cette map.

Install The dwa_local_planner package
```bash
sudo apt-get install ros-noetic-dwa-local-planner
```

- Contrôler le robot en téléopération (clavier)
launch Teleopration Node
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```
- Contrôler le robot en lui donnant un nav goal 2D
to send a navigation goal from RViz, select the 2D Nav Goal tool from the toolbar located at the top, then click somewhere on the map to set position or click and drag to set position and orientation.

## 2. Multiplexeur de commande.
## 3. Téléopération à distance.

Requirements:

Install Mosquitto as MQTT message Broker

```bash
sudo apt install mosquitto mosquitto-clients libmosquitto-dev
```

Also install virtualenv:

```bash
sudo apt-get install python-virtualenv
```

- compile python code :

```bash
cd src/
virtualenv .
source bin/activate
pip install -r requirements.txt
```

- Test the remote control of Turtlebot via MQTT example :

In a first Terminal :

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

In another Terminal Launch the QMTT broker:

```bash
mosquitto -v

```

in another Terminal 
```bash
./Script_Teleop_MQTT.py
```
In another Terminal launch the QMTT subscriber and pulisher of ROS TOPIC "/cmd_vel" to control Linear velocity and Angular Velocity of Turtlebot
```bash
./MQTT_Listener.py
```

## 4. Envoi de Goal déterminé par un tag visuel

- Requirements:

Install OpenCV 
```bash
pip install opencv-python
```
Install ar_track_alvar package:

```bash
install $ sudo apt-get install ros-[YOURDISTRO]-ar-track-alvar
```


## 5. Utilisation des caractéristiques visuelles (bonus)
## 6. Utilisation d’une capture d’image (bonus)
## Authors

- [@Hassene TEKAYA]

