
#include "RobotNodeActuator.h"
#include "../VirtualRobotException.h"

namespace VirtualRobot
{

    RobotNodeActuator::RobotNodeActuator(RobotNodePtr node)
    {
        robotNode = node;
    }

    RobotNodeActuator::~RobotNodeActuator()
    {
    }

    void RobotNodeActuator::updateVisualizationPose(const Eigen::Matrix4f& pose, bool updateChildren)
    {
        robotNode->updateVisualizationPose(pose, updateChildren);
    }

    void RobotNodeActuator::updateVisualizationPose(const Eigen::Matrix4f& pose, float jointValue, bool updateChildren)
    {
        robotNode->updateVisualizationPose(pose, jointValue, updateChildren);
    }

    void RobotNodeActuator::updateJointAngle(float jointValue)
    {
        robotNode->setJointValueNoUpdate(jointValue);
    }

    VirtualRobot::RobotNodePtr RobotNodeActuator::getRobotNode()
    {
        return robotNode;
    }


} // namespace
