# Turtlesim motion

Install turtlesim package

	$ sudo apt-get install ros-$(rosversion -d)-turtlesim
	
Git clone the Repository into your workspace

	$ git clone https://github.com/AshwinDisa/turtlesim_motion.git	
	
Source your workspace where you cloned the repository

	$ gedit .bashrc
	
Add source ~/(YOUR_WORKSPACE_NAME)/devel/setup.bash at the end of the text file
	
Start roscore
	
	$ roscore

Run turtlesim
	
	$ rosrun turtlesim turtlesim_node
	
Run a script for example the spiral_node

	$ rosrun turtlesim_motion turtlesim_spiral_node

	

	

	

	
	




