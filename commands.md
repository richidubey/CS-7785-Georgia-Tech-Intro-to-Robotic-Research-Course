7785
Levi 

ssh burger@192.168.0.121
pass: burger


On both:

————— 

ros2 daemon stop  
ros2 daemon start

echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY

sudo ufw status
sudo ufw disable

——————

source /opt/ros/humble/setup.bash

export TURTLEBOT3_MODEL=burger  
export ROS_DOMAIN_ID=54  
export ROS_LOCALHOST_ONLY=0  

 printenv|grep ROS

ros2 node list
ros2 topic list


ros2 pkg executables 

——————————————————

On Robot:  
ros2 launch turtlebot3_bringup robot.launch.py  

ros2 launch turtlebot3_bringup camera_robot.launch.py  




On Ubuntu

echo 'source ~/turtlebot3_ws/install/setup.bash' >> ~/.Basic  
source ~/.bashrc


ros2 run turtlebot3_teleop teleop_keyboard

ros2 launch turtlebot3_bringup camera_robot.launch.py

ros2 run rqt_image_view rqt_image_view


source install/setup.bash   
richi@richidubeyvm:~/repos/cs7785$ ros2 pkg executables | grep object_tracking    
object_tracking select_object   

ros2 topic echo /test_topic   

ros2 topic pub /test_topic std_msgs/String "data: Hello"    

———    
Turtlesim



Links: 
http://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html 


https://emanual.robotis.com/docs/en/platform/turtlebot3/bringup/#bringup



Publisher Subscriber:    
https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html 


To Watch  
https://eater.net/quaternions
