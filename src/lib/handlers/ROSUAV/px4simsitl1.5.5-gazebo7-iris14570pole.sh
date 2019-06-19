echo "this script will first make sure gazebo 7 is in use, \n then launch gazebo and dronkit example"
rst=`ls -l /usr/local/bin/gazebo|grep 7.4`
chrlen=${#rst}
if [ ! $chrlen -gt 0 ]; then
	echo "gazebo 7.4 required, exiting..."
	~/bin/usegazebo7.sh
fi
cd ~/Downloads/px4-1.5.5/Firmware
#make posix gazebo; 
#make posix_sitl_default; 
cd ~/Downloads/px4-1.5.5/Firmware/Tools

#--1025/18 this works now, if not, reset Firmware/Tools from the tar ball
gnome-terminal -x $SHELL -ic "term_title px4#1sysid2; cd ~/Downloads/px4-1.5.5/Firmware/Tools; /home/student/Downloads/px4-1.5.5/Firmware/build_posix_sitl_default/src/firmware/posix/px4 /home/student/Downloads/px4-1.5.5/Firmware /home/student/Downloads/px4-1.5.5/Firmware/posix-configs/SITL/init/ekf2/iris-14570"
#gnome-terminal -x $SHELL -ic "term_title px4#2sysid3; /home/student/Downloads/px4-1.5.5/Firmware/build_posix_sitl_default/src/firmware/posix/px4 /home/student/Downloads/px4-1.5.5/Firmware /home/student/Downloads/px4-1.5.5/Firmware/posix-configs/SITL/init/ekf2/iris-14571"
exit 
echo "world name: "
echo $1
gnome-terminal -x $SHELL -ic "echo $1; echo worldname; bash"
if [ $1 = "iris_cam2pole.world" ]
then 
	#gnome-terminal -x $SHELL -ic "echo true; bash"
	#exit
	gnome-terminal -x $SHELL -ic "cd ~/Downloads/px4-1.5.5/Firmware/Tools/sitl_gazebo; rosrun gazebo_ros gzserver --verbose worlds/iris_cam2pole.world"
elif [  $1 = "uavcam_pole_test.world" ]
then
	gnome-terminal -x $SHELL -ic "cd ~/Downloads/px4-1.5.5/Firmware/Tools/sitl_gazebo; rosrun gazebo_ros gzserver --verbose worlds/uavcam_pole_test.world"

elif [  $1 = "igvc_final.world" ]
then
	gnome-terminal -x $SHELL -ic "cd ~/Downloads/px4-1.5.5/Firmware/Tools/sitl_gazebo; rosrun gazebo_ros gzserver --verbose worlds/igvc_final.world"
else
        if [ ! -f "/home/student/Downloads/px4-1.5.5/Firmware/Tools/sitl_gazebo/worlds/$1" ]; then
                echo "file not found"
		gnome-terminal -x $SHELL -ic "cd ~/Downloads/px4-1.5.5/Firmware/Tools/sitl_gazebo; rosrun gazebo_ros gzserver --verbose worlds/iris_cam2.world"
        else
	
		#gnome-terminal -x $SHELL -ic "cd ~/Downloads/px4-1.5.5/Firmware/Tools/sitl_gazebo; rosrun gazebo_ros gzserver --verbose worlds/$1"
	fi

	#gnome-terminal -x $SHELL -ic "echo false; bash"
	#exit
fi
	
#--1025/18 this works now

#{--10/25/18 this however work
# sitl_run.sh run both px4 binary and jmavsim.
#./sitl_run_hacking.sh /home/student/Downloads/px4-1.5.5/Firmware/build_posix_sitl_default/src/firmware/posix/px4 posix-configs/SITL/init/ekf2 none gazebo iris /home/student/Downloads/px4-1.5.5/Firmware /home/student/Downloads/px4-1.5.5/Firmware/build_posix_sitl_default
#}--10/25/18 this however work

#gnome-terminal -e "/home/student/Downloads/ardupilot-3.4.5/ardupilot/Tools/autotest/sim_vehicle.py -f gazebo-iris -D --console --map" &
#cd ~/Downloads/gazebo/gazebo-ardupilot
#gnome-terminal -e "gazebo --verbose worlds/iris_arducopter_demo.world" &
#cd ~/dronekit-python/examples/simple_goto
#python simple_goto.py --connect udp:localhost:14550 --gcs udpout:localhost:14560
