6/20/19 ROS ROSUAV driving command execution flow.

	after click start in the simulation tab, the automata engine
	start to send movement command, which will be routed to
	RosDriveHandler.py, which call RosLocomotionCommandHandler.py.
	RosLocomotionCommandHandler.py publish the Twist msg on a topic
	//cmd_vel_mux/input/navi, which is interplated by the gazebo plugin
	code

	modify and debugging RosDriveHandler.py, note if there are syntax error on this
	file, it will not show error like usual in the xterm window... so testing the code
	is a bit trick, 

	RosDriveHandler.py: init func subscribe to /mavros/state, the call back check
	if it is armed and takenoff. if not, call arming and takeoff service. notice takeoff
	service should not be called too freq, we put a 10 sec delay between takeoff cmd.

	after click start button at the simulation window, setVelocity() will be called
	repeatedly. this need to check that proper RosPoseHandler.py is coded for UAV.

6/19/19 tbd:
	ROSUAV/
		modify movement code for uav, translate to mavros
		service call, add arm and takeoff in the init handler
6/19/19 iris gimbal not attached bug
	when take off, the gimbal stay on the ground.
	turn out that iris_px4_standoff_demo_cam1/model.sdf hard coded
	the gimbal joint and require parent link to be iris_demo_cam
	a dirty fix is when spawning the model, the name of the model must be
	iris_demo_cam.
	this is not a problem in iris_cam2_car.world, the name of the model and joint change
	automatically
	
	Good fix: modify ~/.gazebo/models/iris_px4_standoff_demo_cam1
		and ~/.gazebo/models/iris_px4_standoff_demo_cam2
		to remove the reference to the model name in the join
	
	relevent file uavspawn.launch.xml, gazebo_ros spawn_model

6/19/19  jw: how to worldFile are generated and used: 
        ROSUAV/
        worldFile must = ltlmop_map.world to have region drawed. The worldFile is specified at the GUI when configure simulation. prior to today the specified worldfile is always created and existing info wiped. this is changed such that if not ltlmop_map.world, then the file should not be created and assumed existing so a user provide world file can be used.  6/18/19 '

6/18/19 jw: creat RosInitHandler_UAV.py to lauch gazebo and spawn iris drone
        ROS/ 
 rosrun gazebo_ros spawn_model --unpause -sdf -file ~/.gazebo/models/bowl/model.sdf -model Bowl
this version is for UAV. the one in ROS/ use turtlebot

3/19/19 jw: when convert region info to gazebo world file, the
        obstacles (polygon) are converted to box. this is wrong, so for now
        disable the conversion of obstacles.

        turtlebot was spawned in a location, in pixel scale, that must
        be scaled to the size the floor plan.

        the region are converted to a png file, which is used to render

