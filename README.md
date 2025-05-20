Para la ejecución del codigo se dispone de dos métodos:

	-Opción 1:

	Descargar la siguiente máquina virtual: https://nextcloud.citius.usc.es/s/2oJ8AXpRxKmzKLR
	Contraseña: robotica
	

	-Opción 2:

	ROS: Instala la versión adecuada para tu sistema operativo:

	    ROS Kinetic (Ubuntu 16.04)
	    ROS Melodic (Ubuntu 18.04)

	Gazebo: Incluido en la instalación de ROS.
	Python 2.7.12: Versión ya instalada por defecto en las distribuciones compatibles con ROS.

	-Librerias:
		import rospy
		import numpy as np
		import random  
		import json
		import tf
	-Dependencias ROS especificadas en el archivo package.xml:

		<build_depend>geometry_msgs</build_depend>
		<build_depend>nav_msgs</build_depend>
		<build_depend>rospy</build_depend>
		<build_depend>sensor_msgs</build_depend>
		<build_depend>std_msgs</build_depend>
		<build_depend>tf</build_depend>
		<build_depend>visualization_msgs</build_depend>

	-Comunes:
	Debes introducir el paquete enviado y el subpaquete de complementos en el catkin_ws

Método de lanzado 1:

	Mapa 1:
		roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch
		rosrun rviz rviz ( cargar configuracion de : /home/robotica/catkin_ws/src/my_slam_package/config)
		rosrun my_slam_package ekf_slam.py
	Mapa 2:
		roslaunch turtlebot3_gazebo turtlebot3_stage_2.launch
		rosrun rviz rviz ( cargar configuracion de : /home/robotica/catkin_ws/src/my_slam_package/config)
		rosrun my_slam_package ekf_slam.py

	Mapa 3:
		roslaunch turtlebot3_gazebo turtlebot3_world.launch
		rosrun rviz rviz ( cargar configuracion de : /home/robotica/catkin_ws/src/my_slam_package/config)
		rosrun my_slam_package ekf_slam.py

	Mapa 4:
		roslaunch turtlebot3_gazebo turtlebot3_stage_4.launch
		rosrun rviz rviz ( cargar configuracion de : /home/robotica/catkin_ws/src/my_slam_package/config)
		rosrun my_slam_package ekf_slam.py



Método de lanzado 2:

	Mapa 1:roslaunch my_slam_package slam_sim_2.launch 
	Mapa 2:roslaunch my_slam_package slam_sim_3.launch 
	Mapa 3:roslaunch my_slam_package slam_sim.launch
	Mapa 4:roslaunch my_slam_package slam_sim_4.launch 



