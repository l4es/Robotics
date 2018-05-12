#include <Eigen/Geometry>
#include "GenericIKSolver.h"
#include "../Robot.h"
#include "../VirtualRobotException.h"
#include "../Nodes/RobotNode.h"
#include "../VirtualRobotException.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include "../Obstacle.h"
#include "../RobotConfig.h"
#include "../CollisionDetection/CollisionChecker.h"
#include "../CollisionDetection/CDManager.h"

#include <algorithm>

using namespace Eigen;

namespace VirtualRobot
{

    GenericIKSolver::GenericIKSolver(RobotNodeSetPtr rns, JacobiProvider::InverseJacobiMethod invJacMethod) :
        IKSolver(rns)
    {
        this->invJacMethod = invJacMethod;
        _init();
    }

    bool GenericIKSolver::solve(const Eigen::Matrix4f& globalPose, CartesianSelection selection, int maxLoops)
    {
        jacobian->setGoal(globalPose, tcp, selection, maxErrorPositionMM, maxErrorOrientationRad);
        jacobian->checkImprovements(true);

        RobotConfigPtr bestConfig(new RobotConfig(rns->getRobot(), "bestConfig"));
        rns->getJointValues(bestConfig);
        float bestError = jacobian->getMeanErrorPosition();

        // first check reachability
        if (!checkReachable(globalPose))
        {
            return false;
        }

        // first run: start with current joint angles
        if (trySolve())
        {
            return true;
        }

        // if here we failed
        for (int i = 1; i < maxLoops; i++)
        {
            setJointsRandom();

            if (trySolve())
            {
                return true;
            }

            float currentError = jacobian->getMeanErrorPosition();

            if (currentError < bestError)
            {
                rns->getJointValues(bestConfig);
                bestError = jacobian->getMeanErrorPosition();
            }
        }

        // set best config
        rns->setJointValues(bestConfig);
        return false;
    }

    VirtualRobot::GraspPtr GenericIKSolver::solve(ManipulationObjectPtr object, CartesianSelection selection /*= All*/, int maxLoops)
    {
        return IKSolver::solve(object, selection, maxLoops);
    }

    bool GenericIKSolver::solve(ManipulationObjectPtr object, GraspPtr grasp, CartesianSelection selection /*= All*/, int maxLoops)
    {
        return IKSolver::solve(object, grasp, selection, maxLoops);
    }

    void GenericIKSolver::setJointsRandom()
    {
        std::vector<float> jv;
        float rn = 1.0f / (float)RAND_MAX;

        for (unsigned int i = 0; i < rns->getSize(); i++)
        {
            RobotNodePtr ro =  rns->getNode(i);
            float r = (float)rand() * rn;
            float v = ro->getJointLimitLo() + (ro->getJointLimitHi() - ro->getJointLimitLo()) * r;
            jv.push_back(v);
        }

        RobotPtr rob = rns->getRobot();
        rob->setJointValues(rns, jv);

        if (translationalJoint)
        {
            translationalJoint->setJointValue(initialTranslationalJointValue);
        }
    }
    void GenericIKSolver::setupTranslationalJoint(RobotNodePtr rn, float initialValue)
    {
        translationalJoint = rn;
        initialTranslationalJointValue = initialValue;
    }

    DifferentialIKPtr GenericIKSolver::getDifferentialIK()
    {
        return jacobian;
    }

    bool GenericIKSolver::trySolve()
    {

        if (jacobian->solveIK(jacobianStepSize, 0.0, jacobianMaxLoops))
        {
            if (cdm)
            {
                if (!cdm->isInCollision())
                {
                    return true;
                }
            }
            else
            {
                return true;
            }
        }

        return false;
    }

    void GenericIKSolver::_init()
    {
        jacobian.reset(new DifferentialIK(rns, coordSystem, invJacMethod));
        jacobianStepSize = 0.3f;
        jacobianMaxLoops = 50;

    }

    void GenericIKSolver::setupJacobian(float stepSize, int maxLoops)
    {
        this->jacobianStepSize = stepSize;
        this->jacobianMaxLoops = maxLoops;
    }

    bool GenericIKSolver::_sampleSolution(const Eigen::Matrix4f& globalPose, CartesianSelection selection, int maxLoops /*= 1 */)
    {
        //setJointsRandom();
        return solve(globalPose, selection, maxLoops);
    }

    void GenericIKSolver::setVerbose(bool enable)
    {
        verbose = enable;
        jacobian->setVerbose(verbose);
    }


} // namespace VirtualRobot
