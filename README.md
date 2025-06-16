# PEKKA_gz_sim
Simulation of Crop lane following in Gazebo Harmonic 

1. Install Gazebo Crop field generator
   https://github.com/FieldRobotEvent/virtual_maize_field.git

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y


in case of gazebo show the old cache 
check ps aux | grep gz
kill -9
