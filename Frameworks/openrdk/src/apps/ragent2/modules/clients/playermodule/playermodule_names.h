#define PROPERTY_ROBOT_CAMERA_DATA "cameraData"
#define PROPERTY_ROBOT_TAG_DATA "tagData"
#define PROPERTY_PLAYERSERVER_HOST "params/serverHost"
#define PROPERTY_PLAYERSERVER_PORT "params/serverPort"
#define PROPERTY_SIMSERVER_HOST "params/simServerHost"
#define PROPERTY_SIMSERVER_PORT "params/simServerPort"
#define PROPERTY_SLEEP "params/sleepTime"
#define PROPERTY_ROBOT_CLASS "params/robotClass"
#define PROPERTY_ENABLE_LASER "params/enableLaser"
#define PROPERTY_ENABLE_SONAR "params/enableSonar"
#if ! defined PLAYER_VERSION_LT_2_1 && ! defined PLAYER_VERSION_LT_3
#define PROPERTY_ENABLE_RANGER "params/enableRanger"
#endif
#define PROPERTY_ENABLE_CAMERA "params/enableCamera"
#define PROPERTY_ENABLE_FIDUCIAL "params/enableFiducial"
#define PROPERTY_ENABLE_MOTORS "params/enableMotors"
#define PROPERTY_ENABLE_GROUND_TRUTH "params/enableGroundTruth"
#define PROPERTY_GROUND_TRUTH_POSE "out/groundTruthPose"
#define PROPERTY_ROBOT_NAME "params/robotName"
#define PROPERTY_WORLD_CENTER "params/centerOfWorld"

#define PROPERTY_CMD_SIMULATION_SET_POSE "cmds/simulationSetPose"
#define PROPERTY_CMD_SIMULATION_SET_POSE_ARG "cmds/simulationSetPoseArg"

#define PROPERTY_USE_GROUND_TRUTH_FOR_LASER "params/useGroundTruthForLaser"

