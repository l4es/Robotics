#include "Constraint.h"

using namespace VirtualRobot;

Constraint::Constraint(const RobotNodeSetPtr &nodeSet) :
    JacobiProvider(nodeSet, JacobiProvider::eSVDDamped),
    lastError(-1),
    lastLastError(-1)
{

}

void Constraint::initialize()
{
    lastError = -1;
    lastLastError = -1;
}

bool Constraint::getRobotPoseForConstraint(Eigen::Matrix4f &pose)
{
    // No change in global pose required
    return false;
}

float Constraint::getErrorDifference()
{
    Eigen::VectorXf e = getError(1);
    lastLastError = lastError;
    lastError = e.norm();

    if(lastLastError <= 0)
    {
        return 0;
    }

    return lastLastError - lastError;
}
