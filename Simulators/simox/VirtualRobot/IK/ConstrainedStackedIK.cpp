#include "ConstrainedStackedIK.h"

using namespace VirtualRobot;

ConstrainedStackedIK::ConstrainedStackedIK(RobotPtr &robot, const RobotNodeSetPtr &nodeSet, float stepSize, int maxIterations, JacobiProvider::InverseJacobiMethod method) :
    ConstrainedIK(robot, maxIterations),
    nodeSet(nodeSet),
    method(method),
    stepSize(stepSize)
{
}

bool ConstrainedStackedIK::initialize()
{
    ik.reset(new StackedIK(nodeSet, method));
    jacobians.clear();

    for(auto &constraint : constraints)
    {
        jacobians.push_back(constraint);
    }

    return ConstrainedIK::initialize();
}

bool ConstrainedStackedIK::solveStep()
{
    THROW_VR_EXCEPTION_IF(!ik, "IK not initialized, did you forget to call initialize()?");

    Eigen::VectorXf jointValues;
    Eigen::VectorXf delta = ik->computeStep(jacobians, stepSize);

    nodeSet->getJointValues(jointValues);
    nodeSet->setJointValues(jointValues + delta);

    return true;
}
