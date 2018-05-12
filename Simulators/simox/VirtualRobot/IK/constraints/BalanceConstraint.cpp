#include "BalanceConstraint.h"

#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoSelection.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoTransform.h>

using namespace VirtualRobot;

BalanceConstraint::BalanceConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SceneObjectSetPtr &contactNodes,
                                     float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates, bool considerCoMHeight) :
    Constraint(joints)
{
    initialize(robot, joints, bodies, contactNodes, tolerance, minimumStability, maxSupportDistance, supportPolygonUpdates, considerCoMHeight);
}

BalanceConstraint::BalanceConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SupportPolygonPtr &supportPolygon,
                                     float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates, bool considerCoMHeight) :
    Constraint(joints)
{
    initialize(robot, joints, bodies, supportPolygon->getContactModels(), tolerance, minimumStability, maxSupportDistance, supportPolygonUpdates, considerCoMHeight);
}

void BalanceConstraint::initialize(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SceneObjectSetPtr &contactNodes,
                                   float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates, bool considerCoMHeight)
{
    this->joints = joints;
    this->bodies = bodies;
    this->minimumStability = minimumStability;
    this->maxSupportDistance = maxSupportDistance;
    this->tolerance = tolerance;
    this->supportPolygonUpdates = supportPolygonUpdates;
    this->considerCoMHeight = considerCoMHeight;

    supportPolygon.reset(new SupportPolygon(contactNodes));

    VR_INFO << "COM height:" << height << endl;
    int dim = 2;
    if (considerCoMHeight)
    {
        dim = 3;
        setCoMHeight(height);
    }
    else
    {
        height = robot->getCoMGlobal()(2);
        setCoMHeight(height);
    }
    comIK.reset(new CoMIK(joints, bodies, RobotNodePtr(), dim));

    updateSupportPolygon();

    initialized = true;
}

void BalanceConstraint::updateSupportPolygon()
{
    supportPolygon->updateSupportPolygon(maxSupportDistance);

    MathTools::ConvexHull2DPtr convexHull = supportPolygon->getSupportPolygon2D();
    THROW_VR_EXCEPTION_IF(!convexHull, "Empty support polygon for balance constraint");

    Eigen::Vector2f supportPolygonCenter = MathTools::getConvexHullCenter(convexHull);

    if (considerCoMHeight)
    {
        Eigen::Vector3f goal;
        goal.head(2) = supportPolygonCenter;
        goal(2) = height;
        comIK->setGoal(goal, tolerance);
        VR_INFO << "CoM Height of Inversed Robot set to:" << goal(2) << endl;
    } else
        comIK->setGoal(supportPolygonCenter, tolerance);
}

void BalanceConstraint::setCoMHeight(float currentheight)
{
    height = currentheight;
}

Eigen::MatrixXf BalanceConstraint::getJacobianMatrix()
{
    return comIK->getJacobianMatrix();
}

Eigen::MatrixXf BalanceConstraint::getJacobianMatrix(SceneObjectPtr tcp)
{
    return comIK->getJacobianMatrix(tcp);
}

Eigen::VectorXf BalanceConstraint::getError(float stepSize)
{
    float stability = supportPolygon->getStabilityIndex(bodies, supportPolygonUpdates);

    if(supportPolygonUpdates)
    {
        updateSupportPolygon();
    }

    if(stability < minimumStability)
    {
        return comIK->getError(stepSize);
    }
    else if(considerCoMHeight)
    {
        return Eigen::Vector3f(0,0,0);
    }
    else
    {
        return Eigen::Vector2f(0,0);
    }
}

bool BalanceConstraint::checkTolerances()
{
    return (supportPolygon->getStabilityIndex(bodies, supportPolygonUpdates) >= minimumStability);
}

Eigen::Vector3f BalanceConstraint::getCoM()
{
    return bodies->getCoM();
}

SupportPolygonPtr BalanceConstraint::getSupportPolygon()
{
    return supportPolygon;
}

std::string BalanceConstraint::getConstraintType()
{
    return "Balance(" + joints->getName() + ")";
}


