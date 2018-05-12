#include "usarsimclientmodule.h"

namespace RDK2 { namespace RAgent {

void UsarSimClientModule::computeWheelSpeed(Session *session, double cmdSpeed, double cmdJog,
	double& leftWheelSpeed, double& rightWheelSpeed)
{
	double wheelRadius = 1, wheelDistance = 1;
	string robotClass = session->getString(PROPERTY_ROBOT_CLASS);
	if (robotClass.find("Tarantula", 0) != string::npos || robotClass.find("kenaf", 0) != string::npos) {
		wheelRadius = TARANTULA_MIN_WHEEL_RADIUS;
		wheelDistance = TARANTULA_LENGTH;
	}
	else if (robotClass.find("P2AT", 0) != string::npos) {
		wheelRadius = P2AT_WHEEL_RADIUS;
		wheelDistance = P2AT_DIAMETER;
	}
	else if (robotClass.find("P2DX", 0) != string::npos) {
		wheelRadius = P2DX_WHEEL_RADIUS;
		wheelDistance = P2DX_DIAMETER;
	}
	else if (robotClass.find("Zerg", 0) != string::npos) {
		wheelRadius = ZERG_WHEEL_RADIUS;
		wheelDistance = ZERG_WHEEL_DISTANCE;
	}
	else if (robotClass.find("ComStation", 0) != string::npos) {
		wheelRadius = 1;
		wheelDistance = 1;
	}
	else {
		RDK_ERROR_PRINTF("Unknown robot class: %s, using P2AT geometry", robotClass.c_str());
		wheelRadius = P2AT_WHEEL_RADIUS;
		wheelDistance = P2AT_DIAMETER;
	}
	
	leftWheelSpeed  = (cmdSpeed - (cmdJog * wheelDistance / 2. )) / wheelRadius;
	rightWheelSpeed = (cmdSpeed + (cmdJog * wheelDistance / 2. )) / wheelRadius;
}

}}