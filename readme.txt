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

