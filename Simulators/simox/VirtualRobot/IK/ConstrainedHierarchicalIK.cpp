#include "ConstrainedHierarchicalIK.h"

using namespace VirtualRobot;

ConstrainedHierarchicalIK::ConstrainedHierarchicalIK(RobotPtr &robot, const RobotNodeSetPtr &nodeSet, float stepSize, int maxIterations, float stall_epsilon, float raise_epsilon) :
    ConstrainedIK(robot, maxIterations, stall_epsilon, raise_epsilon),
    nodeSet(nodeSet),
    stepSize(stepSize)
{
}

bool ConstrainedHierarchicalIK::initialize()
{
    ik.reset(new HierarchicalIK(nodeSet));
    jacobians.clear();

    for(auto &constraint : constraints)
    {
        jacobians.push_back(constraint);
    }

    return ConstrainedIK::initialize();
}

bool ConstrainedHierarchicalIK::solveStep()
{
    THROW_VR_EXCEPTION_IF(!ik, "IK not initialized, did you forget to call initialize()?");

    Eigen::VectorXf jointValues;
    Eigen::VectorXf delta = ik->computeStep(jacobians, stepSize);

    // Check the stall condition
    if(lastDelta.rows() > 0 && (delta - lastDelta).norm() < stallEpsilon)
    {
        VR_INFO << "Constrained IK failed due to stall condition" << std::endl;
        return false;
    }
    lastDelta = delta;

    // Check if any of the hard constraints has increased error
    for(auto &constraint : constraints)
    {
        if(hardConstraints[constraint] && (constraint->getErrorDifference() < -raiseEpsilon))
        {
            VR_INFO << "Constrained IK failed due to error raise for hard constraint " << constraint->getConstraintType() << std::endl;
            return false;
        }
    }

    nodeSet->getJointValues(jointValues);
    nodeSet->setJointValues(jointValues + delta);

    return true;
}
