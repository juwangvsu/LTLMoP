RobotName: # The name of the robot
Hierarchical

Type: # Robot type
Hierarchical

InitHandler: # Robot default init handler with default argument values
Hierarchical.RosHierarchicalInitHandler()

PoseHandler: # Robot default pose handler with default argument values
share.Pose.GazeboPoseHandler(modelName="mobile_base")

SensorHandler: # Robot default sensors handler with default argument values
Hierarchical.RosHierarchicalSensorHandler()

ActuatorHandler: # Robot default actuator handler wit hdefault argument values
#Hierarchical.RosHierarchicalActuatorHandler()

MotionControlHandler: # Robot default motion control handler with default argument values
share.MotionControl.AbstractHandler()

DriveHandler: # Robot default drive handler with default argument values
Hierarchical.RosHierarchicalDriveHandler(d=.3)

LocomotionCommandHandler: # Robot default locomotion command handler with default argument values
Hierarchical.RosHierarchicalLocomotionCommandHandler(velocityTopic='/cmd_vel_mux/input/navi')
