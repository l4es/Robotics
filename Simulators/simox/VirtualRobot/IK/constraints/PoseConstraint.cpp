#include "PoseConstraint.h"

#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>

using namespace VirtualRobot;

PoseConstraint::PoseConstraint(const RobotPtr &robot, const RobotNodeSetPtr &nodeSet, const RobotNodePtr &eef, const Eigen::Matrix4f &target, IKSolver::CartesianSelection cartesianSelection,
                               float tolerancePosition, float toleranceRotation) :
    Constraint(nodeSet),
    robot(robot),
    nodeSet(nodeSet),
    eef(eef),
    target(target),
    cartesianSelection(cartesianSelection),
    tolerancePosition(tolerancePosition),
    toleranceRotation(toleranceRotation)
{
    ik.reset(new DifferentialIK(nodeSet));
    ik->setGoal(target, eef, cartesianSelection, tolerancePosition, toleranceRotation);
    initialized = true;
}

void PoseConstraint::setVisualization(const SceneObjectSetPtr &visualizationNodeSet)
{
    this->visualizationNodeSet = visualizationNodeSet;
}

Eigen::MatrixXf PoseConstraint::getJacobianMatrix()
{
    return ik->getJacobianMatrix(eef, cartesianSelection);
}

Eigen::MatrixXf PoseConstraint::getJacobianMatrix(SceneObjectPtr tcp)
{
    if(tcp->getName() != eef->getName())
    {
        VR_WARNING << "EndEffectorPoseConstraing Jacobina calculation for differing EEF ('" << tcp->getName() << "' instead of '" << eef->getName() << "')" << std::endl;
    }

    return ik->getJacobianMatrix(tcp);
}

Eigen::VectorXf PoseConstraint::getError(float stepSize)
{
    return ik->getError(stepSize);
}

bool PoseConstraint::checkTolerances()
{
    return ik->checkTolerances();
}

bool PoseConstraint::getRobotPoseForConstraint(Eigen::Matrix4f &pose)
{
    if(robot->getRootNode()->getName() == eef->getName())
    {
        // If the end effector to move equals the robot root, we initially move the whole robot to this
        // position in order to satisfy this constraint
        pose = target;
        return true;
    }

    return false;
}

std::string PoseConstraint::getConstraintType()
{
    return "Pose(" + eef->getName() + ")";
}

const Eigen::Matrix4f &PoseConstraint::getTarget()
{
    return target;
}

void PoseConstraint::updateTarget(const Eigen::Matrix4f &newTarget)
{
    target = newTarget;
    ik->setGoal(target, eef, cartesianSelection, tolerancePosition, toleranceRotation);
}

