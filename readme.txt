6/19/19 tbd:
	ROSUAV/
		modify movement code for uav, translate to mavros
		service call, add arm and takeoff in the init handler

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

