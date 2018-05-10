#include "ConstrainedIK.h"

using namespace VirtualRobot;

ConstrainedIK::ConstrainedIK(RobotPtr &robot, int maxIterations, float stall_epsilon, float raise_epsilon) :
    robot(robot),
    maxIterations(maxIterations),
    currentIteration(0),
    running(false),
    stallEpsilon(stall_epsilon),
    raiseEpsilon(raise_epsilon)
{

}

void ConstrainedIK::addConstraint(const ConstraintPtr &constraint, int priority, bool hard_constraint)
{
    constraints.push_back(constraint);
    priorities[constraint] = priority;
    hardConstraints[constraint] = hard_constraint;
    std::sort(constraints.begin(), constraints.end(), [this](const ConstraintPtr& lhs, const ConstraintPtr& rhs){return priorities[lhs] > priorities[rhs];});
}

void ConstrainedIK::removeConstraint(const ConstraintPtr &constraint)
{
    auto position = std::find(constraints.begin(), constraints.end(), constraint);

    if(position != constraints.end())
    {
        constraints.erase(position);
        priorities.erase(constraint);
        hardConstraints.erase(constraint);
    }
}

std::vector<ConstraintPtr> ConstrainedIK::getConstraints()
{
    return constraints;
}

bool ConstrainedIK::initialize()
{
    Eigen::Matrix4f P;
    int moves = 0;

    for(auto &constraint : constraints)
    {
        constraint->initialize();
        moves += constraint->getRobotPoseForConstraint(P);
    }

    if(moves == 1)
    {
        // Only one constraint requested to move the robot => No conflicts
        robot->setGlobalPose(P);
    }
    else if(moves > 1)
    {
        VR_ERROR << "Multiple constraints requested to move the robot" << std::endl;
        return false;
    }

    running = true;
    currentIteration = 0;
    return true;
}

bool ConstrainedIK::solve(bool stepwise)
{
    while(currentIteration < maxIterations)
    {
        if(!solveStep())
        {
            running = false;
            return false;
        }

        bool goalReached = true;
        for(auto &constraint : constraints)
        {
            if(hardConstraints[constraint] && !constraint->checkTolerances())
            {
                goalReached = false;
            }
        }

        if(goalReached)
        {
            running = false;
            return true;
        }

        currentIteration++;

        if(stepwise)
        {
            // Identify this case via running variable
            return false;
        }
    }

    running = false;
    return false;
}

void ConstrainedIK::setMaxIterations(int maxIterations)
{
    this->maxIterations = maxIterations;
}

int ConstrainedIK::getMaxIterations()
{
    return maxIterations;
}

bool ConstrainedIK::getRunning()
{
    return running;
}

int ConstrainedIK::getCurrentIteration()
{
    return currentIteration;
}

