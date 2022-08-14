
# Test De Recrutement iFollow

Dans ce fichier Readme, nous avons mis les réponses au exercices qui permettent au recruteur d'évaluer les capacités du candidat à intégrer et utiliser
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

Method 1 : Graphic method from RVIZ

Launch turtlebot3_navigation package

```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```

To send a navigation goal from RViz, select the 2D Nav Goal tool from the toolbar located at the top, then click somewhere on the map to set position or click and drag to set position and orientation.

Method 2 : Command Line

Choose values of position and Orientation and publishing to the topic /move_base_simple/goal

```bash
rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.5, y: 1.4, z: 0.0}, orientation: {w: 1.0}}}'
```


## 2. Multiplexeur de commande.

Setting up the sel_cmdvel topic to switch between /cmd_local and /cmd_web :

```bash
rosrun topic_tools mux sel_cmdvel cmd_local cmd_web mux:=mux_cmdvel
```

Then using mux_select to select cmd_local:

```bash
rosrun topic_tools mux_select mux_cmdvel cmd_local
```

Or using mux_select to select cmd_web:

```bash
rosrun topic_tools mux_select mux_cmdvel cmd_web
```


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

- compile python code "Script_Teleop_MQTT.py" and "MQTT_Listener.py":

```bash
cd scripts/
virtualenv .
source bin/activate
pip install -r
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
I used Python bindings for the Apriltags library developed by AprilRobotics.

This library creates two classes that are used to detect apriltags and extract information from
them. Using this module, you can identify all apriltags visible in an
image, and get information about the location and orientation of the
tags

Command to install dt-apriltags
```bash
pip install dt-apriltags
```
I wrote a pyhon script navtogoal.py to detect the 3 AprtilTags (families='tag36h11),I created test_files folder were I put 3 April Tags JPG image .

 The Detector class is a wrapper around the Apriltags functionality. it can be initialized as following:


                at_detector = Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)
-Compile :

- compile python code "nav2goal.py":

```bash
cd scripts/
virtualenv .
source bin/activate
pip install -r
```

Launch test :

```bash
./nav2goal.py
```

- Result:

For Ar_tag_1.JPG : tag_id =  20

For Ar_tag_2.JPG : tag_id =  21 (test failed with current image, image resized to get the result)

For Ar_tag_2.JPG : tag_id =  22


## 5. Utilisation des caractéristiques visuelles (bonus)

Requirements:

```bash
pip install numpy-quaternion
```
I wrote a simple function to transform the AprilTag orienation (as a rotation matrix) to quaternion (array of 4 elements).


```bash
#### TRANSFORM ROTATION MATRIX TO QUATERNION ####                     
def RotationMatrixToQuaternion(m):
  t = np.matrix.trace(m)
  q_array = np.asarray([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
  q = np.quaternion(1,0,0,0)

  if(t > 0):
    t = np.sqrt(t + 1)
    q_array[0] = 0.5 * t
    t = 0.5/t
    q_array[1] = (m[2,1] - m[1,2]) * t
    q_array[2] = (m[0,2] - m[2,0]) * t
    q_array[3] = (m[1,0] - m[0,1]) * t

  else:
    i = 0
    if (m[1,1] > m[0,0]):
        i = 1
    if (m[2,2] > m[i,i]):
        i = 2
    j = (i+1)%3
    k = (j+1)%3

    t = np.sqrt(m[i,i] - m[j,j] - m[k,k] + 1)
    q_array[i] = 0.5 * t
    t = 0.5 / t
    q_array[0] = (m[k,j] - m[j,k]) * t
    q_array[j] = (m[j,i] + m[i,j]) * t
    q_array[k] = (m[k,i] + m[i,k]) * t
    
  q.w = q_array[0]
  q.x = q_array[1]
  q.y = q_array[2]
  q.z = q_array[3]

  return q
####################################################
```

Finaly , I calculated the inverse of the quaterion obtained to have the right orineation of the robot.

```bash
orient_nav_goal = np.quaternion.inverse (RotationMatrixToQuaternion(tag.pose_R))
```

- compile python code "nav2goal_plus_orientation.py":

```bash
cd scripts/
virtualenv .
source bin/activate
pip install -r
```

Launch test :

```bash
./nav2goal_plus_orientation.py
```

- Result for "Ar_tag_1.JPG":

```bash
[INFO] [1660515258.714767, 0.000000]: going to pose id: 20
[INFO] [1660515258.717473, 68806.184000]: going to pose 
position: 
  x: -0.55
  y: -1.65
  z: 0.0
orientation: quaternion(0.960249420417204, -0.000441925911480079, -0.00091205388331065, 0.279141583157414)
```
## 6. Utilisation d’une capture d’image (bonus)

this line n°84 in "nav2goal_plus_orientation.py"

```bash
 img = cv2.imread(test_images_path+'/'+ 'ar_tag_1.JPG', cv2.IMREAD_GRAYSCALE)
 ```
 
 will be replaced by
 
```bash
 rospy.wait_for_service('Trigger')
 rospy.ServiceProxy('Trigger', trigger_srv.srv.Empty)
 cap = cv2.VideoCapture(0)
 ret, frame = cap.read()
 img = cv2.imread(frame, cv2.IMREAD_GRAYSCALE)
  ```

  And we need to create a service called "Trigger" (python script "Trigger_Server.py")
  
  The service can be called this service through the command:

  ``bash
  rosservice call /Trigger
  ```
## Authors

- [@Hassene TEKAYA]

## Running Tests

To run tests, run the following command

```bash
  npm run test
```


