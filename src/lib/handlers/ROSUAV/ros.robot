RobotName: # The name of the robot
ROSUAV

Type: # Robot type
ROSUAV

InitHandler: # Robot default init handler with default argument values
ROSUAV.RosInitHandler(worldFile="ltlmop_map.world", robotPixelWidth=10, robotPhysicalWidth=.4, robotPackage="turtlebot_bringup", robotLaunchFile="minimal.launch")

PoseHandler: # Robot default pose handler with default argument values
ROSUAV.RosPoseHandler(modelName="mobile_base")

SensorHandler: # Robot default sensors handler with default argument values
ROSUAV.RosSensorHandler()

#ActuatorHandler: # Robot default actuator handler wit hdefault argument values
#ROSUAV.RosActuatorHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.VectorControllerHandler()

DriveHandler: # Robot default drive handler with default argument values
ROSUAV.RosDriveHandler(d=.3)

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
ROSUAV.RosLocomotionCommandHandler(velocityTopic='/cmd_vel_mux/input/navi')
