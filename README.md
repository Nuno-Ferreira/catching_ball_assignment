Step 1: Setup URSIM

1: Open URSIM with admin privileges
	cd ~/ursim-3.15.4.106291
	sudo ./start-ursim.sh UR10

2: Wait for it to finish opening, and click through all the error messages, then close it.

3: reopen without admin privs
	./start-ursim.sh UR10

------------------------------------------------------------------------------------------
Step 2: UR-Ros drivers

In a new terminal (control + shift + T)

Option 1: use provided launch file
	This launch file needs to be edited, but once edited, you can use it easily.
	To Edit: 
		gedit ~/catkin_ws/src/robot_control_simulation/launch/ur_robot_drivers.launch
			change the "robot_ip" parameter to match the URSIM's IP
	To launch:
		roslaunch robot_control_simulation ur_robot_drivers.launch

Option 2: use the pkgs directly
	roslaunch ur_robot_driver ur10_bringup.launch robot_ip:="insert_URSIM's_ip_here"

------------------------------------------------------------------------------------------
Step 3: Start ball traj spawner

In a 3rd terminal (control + shift + T)

1: rosrun robot_control_simulation ball_traj
	This spawns a service that can be called within a script. It will return a message that tells you your distance from it as well as the orientation error.
2: Input either 1 or 2. 
	Inputting 1 will let you practise on just directional error, where having correct cartesian coordinates will randomise the trajectory. Inputting 2 will run the actual ball trajectory and will require the directional error and rotational error to be corrected before another ball is "thrown".


------------------------------------------------------------------------------------------	
Step4: Run code

In a 4th terminal (control + shift + T)

1: python3 assignment_3.py
	You will need to have the file 'assignment_3.py' inside the 'scripts' folder inside the 'robot_control_simulation' package
    Depending on system it might be the case that it needs to be "python assignment_3.py"
    This runs the code and the are terminal messages indicating the progress to the threshold. 
    If you wish it to catch the ball/object again then rerun the command "python3 assignment_3.py"