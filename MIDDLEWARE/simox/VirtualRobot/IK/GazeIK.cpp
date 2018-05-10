#include "GazeIK.h"
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>

#include <algorithm>
#include <float.h>

using namespace VirtualRobot;
using namespace std;


namespace VirtualRobot
{


    GazeIK::GazeIK(RobotNodeSetPtr rns, RobotNodePrismaticPtr virtualTranslationJoint)
        : rns(rns), virtualTranslationJoint(virtualTranslationJoint)
    {
        nodes = rns->getAllRobotNodes();
        VR_ASSERT(nodes.size() > 0);
        VR_ASSERT(virtualTranslationJoint);
        VR_ASSERT(virtualTranslationJoint->getParent());
        VR_ASSERT(rns->hasRobotNode(virtualTranslationJoint));
        enableJLA = true;
        maxLoops = 50; // nr of seeds for gradient descent
        maxPosError = 5.0f; //mm
        maxGradientDecentSteps = 30;
        randomTriesToGetBestConfig = 40; // nr of random samples that are evaluated to determine the "best" starting point for gradient decent
        verbose = false;
        setupIK();
    }

    void GazeIK::setVerbose(bool v)
    {
        verbose = v;
        setupIK();
    }

    void GazeIK::setup(float maxPosError, int maxLoops, int maxGradientDecentSteps)
    {
        this->maxLoops = maxLoops;
        this->maxPosError = maxPosError;
        this->maxGradientDecentSteps = maxGradientDecentSteps;
        setupIK();
    }

    void GazeIK::enableJointLimitAvoidance(bool enable)
    {
        enableJLA = enable;
        setupIK();
    }

    void GazeIK::setupIK()
    {
        if (!rns || !rns->getTCP() || !virtualTranslationJoint)
        {
            return;
        }

        ikSolver.reset(new HierarchicalIK(rns));
        //ikSolver->setVerbose(verbose);

        // 1. gaze
        ikGaze.reset(new DifferentialIK(rns , RobotNodePtr(), VirtualRobot::JacobiProvider::eSVDDamped));
        ikGaze->setVerbose(verbose);

        //Eigen::VectorXf js(rns->getSize());
        //js.setConstant(1.0f);
        //js(js.rows() - 1) = 0.1f;
        //js.normalize();
        //ikGaze->setJointScaling(js);
        //ikGaze->convertModelScalingtoM(true);


        // 2. jl avoidance
        ikJointLimits.reset();

        if (enableJLA)
        {
            ikJointLimits.reset(new JointLimitAvoidanceJacobi(rns, VirtualRobot::JacobiProvider::eSVDDamped));
        }
    }


    Eigen::VectorXf GazeIK::computeStep(const Eigen::Vector3f &goal, float stepSize)
    {
        VR_ASSERT(ikSolver && ikGaze && virtualTranslationJoint);

        std::vector<JacobiProviderPtr> jacobies;

        //Eigen::Matrix4f currentPose = rns->getTCP()->getGlobalPose(); // the "tcp" of the gaze RNS -> the gaze point which is moved via a virtual translational joint
        Eigen::Matrix4f g;
        g.setIdentity();
        g.block(0, 3, 3, 1) = goal;
        ikGaze->setGoal(g, rns->getTCP(), IKSolver::Position);
        ikGaze->setMaxPositionStep(20.0f);
        Eigen::VectorXf deltaGaze = ikGaze->getError();// getDelta(currentPose, g);

        if (verbose)
        {
            VR_INFO << "ikGaze delta:\n" << deltaGaze.head(3) << endl;
        }

        jacobies.push_back(ikGaze);

        // 2. jl avoidance
        if (ikJointLimits)
        {
            jacobies.push_back(ikJointLimits);
        }

        //compute step

        Eigen::VectorXf delta = ikSolver->computeStep(jacobies, stepSize);
        return delta;
    }

    bool GazeIK::solve(const Eigen::Vector3f &goal, float stepSize)
    {
        if (!ikSolver || !ikGaze || !virtualTranslationJoint)
        {
            return false;
        }
        float bestDist = FLT_MAX;
        std::vector<float> jvBest;

        // initialize the virtualGazeJoint with a guess
        float v = (goal - virtualTranslationJoint->getParent()->getGlobalPose().block(0, 3, 3, 1)).norm();
        virtualTranslationJoint->setJointValue(v);
        //VR_INFO << "virtualTranslationJoint jv:" << v << endl;


        // first run: start with current joint angles
        if (trySolve(goal, stepSize))
        {
            return true;
        }

        VR_INFO << "No solution from current configuration, trying with random seeded configuration" << endl;

        float posDist = getCurrentError(goal);
        jvBest = rns->getJointValues();
        bestDist = posDist;

        // if here we failed
        for (int i = 1; i < maxLoops; i++)
        {
            //VR_INFO << "loop " << i << endl;
            // set rotational joints randomly
            setJointsRandom(goal, randomTriesToGetBestConfig);

            // update translational joint with initial guess
            float v = (goal - virtualTranslationJoint->getParent()->getGlobalPose().block(0, 3, 3, 1)).norm();
            virtualTranslationJoint->setJointValue(v);
            //VR_INFO << "virtualTranslationJoint jv:" << v << endl;


            // check if there is a gradient to the solution
            if (trySolve(goal, stepSize))
            {
                //VR_INFO << "Solution found " << endl;
                return true;
            }

            posDist = getCurrentError(goal);
            if (posDist < bestDist)
            {
                //VR_INFO << "New best solution, dist:" << posDist << endl;
                jvBest = rns->getJointValues();
                bestDist = posDist;
            }
        }

        VR_INFO << "Setting joint values ot best achieved config, dist to target:" << bestDist << endl;
        rns->setJointValues(jvBest);

        return false;
    }


    void GazeIK::setJointsRandom(const Eigen::Vector3f &goal, int bestOfTries)
    {
        if (bestOfTries <= 0)
            bestOfTries = 1;

        float bestDist = FLT_MAX;
        float posDist;
        std::vector<float> jvBest;
        for (int i = 0; i < bestOfTries; i++)
        {
            setJointsRandom();
            posDist = getCurrentError(goal);
            if (posDist < bestDist)
            {
                //VR_INFO << "joints random, best dist:" << posDist << endl;
                bestDist = posDist;
                jvBest = rns->getJointValues();
            }
        }
        rns->setJointValues(jvBest);
    }

    void GazeIK::setJointsRandom()
    {
        if (!rns)
        {
            return;
        }

        std::vector<float> jv;
        float rn = 1.0f / (float)RAND_MAX;

        for (unsigned int i = 0; i < rns->getSize(); i++)
        {
            RobotNodePtr ro =  rns->getNode(i);
            float v = ro->getJointValue();

            if (ro->isRotationalJoint())
            {
                float r = (float)rand() * rn;
                v = ro->getJointLimitLo() + (ro->getJointLimitHi() - ro->getJointLimitLo()) * r;
            }

            jv.push_back(v);
        }

        rns->setJointValues(jv);
    }

    float GazeIK::getCurrentError(const Eigen::Vector3f &goal)
    {
        if (!rns)
        {
            return 0.0f;
        }

        Eigen::Vector3f position = goal - rns->getTCP()->getGlobalPose().block(0, 3, 3, 1);
        return position.norm();
    }

    bool GazeIK::checkTolerances(const Eigen::Vector3f &goal)
    {
        return (getCurrentError(goal) <= maxPosError);
    }

    void GazeIK::applyJLA(const Eigen::Vector3f &goal, int steps, float stepSize)
    {
        float minJLAChange = 1e-6f;
        std::vector<float> jv(nodes.size(), 0.0f);
        std::vector<float> jvBest = rns->getJointValues();
        int step = 0;

        while (step < steps)
        {
            Eigen::VectorXf dTheta = this->computeStep(goal, stepSize);
            if (verbose)
            {
                VR_INFO << "applyJLA step " << step << ", theta:" << dTheta.transpose() << endl;
            }


            for (unsigned int i = 0; i < nodes.size(); i++)
            {
                jv[i] = (nodes[i]->getJointValue() + dTheta[i]);

                // sanity check
                if (boost::math::isnan(jv[i]) || boost::math::isinf(jv[i]))
                {
                    rns->setJointValues(jvBest);
                    VR_WARNING << "Aborting, invalid joint value (nan)" << endl;
                    return;
                }
            }

            rns->setJointValues(jv);

            if (!checkTolerances(goal))
            {
                // reset to last valid setup
                rns->setJointValues(jvBest);
                return;
            }

            float d = dTheta.norm();

            if (d < minJLAChange)
            {
                if (verbose)
                {
                    VR_INFO << "Could not improve result any more with joint limit avoidance tasks (dTheta.norm()=" << d << "), loop:" << step << endl;
                }

                return;
            }

            jvBest = jv;
            step++;
        }
    }

    bool GazeIK::trySolve(const Eigen::Vector3f &goal, float stepSize)
    {
        VR_ASSERT(rns);
        RobotPtr robot = rns->getRobot();
        VR_ASSERT(robot);
        float bestDist = FLT_MAX;
        int jlaSteps = 15;
        float minumChange = 1e-5f;
        std::vector<float> jv(nodes.size(), 0.0f);
        std::vector<float> jvBest = rns->getJointValues();
        int step = 0;
        checkTolerances(goal);

        while (step < maxGradientDecentSteps)
        {
            Eigen::VectorXf dTheta = this->computeStep(goal, stepSize);

            if (verbose)
            {
                VR_INFO << "step " << step << ", theta:" << dTheta.transpose() << endl;
            }

            for (unsigned int i = 0; i < nodes.size(); i++)
            {
                jv[i] = (nodes[i]->getJointValue() + dTheta[i]);

                // sanity check
                if (boost::math::isnan(jv[i]) || boost::math::isinf(jv[i]))
                {
                    VR_WARNING << "Aborting, invalid joint value (nan)" << endl;
                    return false;
                }
            }

            robot->setJointValues(rns, jv);

            // check tolerances
            if (checkTolerances(goal))
            {
                if (verbose)
                {
                    VR_INFO << "Tolerances ok, loop:" << step << endl;
                }

                // try to improve pose by applying some joint limit avoidance steps
                applyJLA(goal, jlaSteps, stepSize);
                return true;
            }

            /*float posDist = getCurrentError(goal);
            if (checkImprovement && posDist>lastDist)
            {
                if (verbose)
                    VR_INFO << "Could not improve result any more (current position error=" << posDist << ", last loop's error:" << lastDist << "), loop:" << step << endl;
                robot->setJointValues(rns,jvBest);
                return false;
            }*/

            float d = dTheta.norm();

            if (d < minumChange)
            {
                if (verbose)
                {
                    VR_INFO << "Could not improve result any more (dTheta.norm()=" << d << "), loop:" << step << endl;
                }

                return false;
            }

            float posDist = getCurrentError(goal);
            if (posDist < bestDist)
            {
                jvBest = jv;
                bestDist = posDist;
            }
            step++;
        }

        if (verbose)
        {
            VR_INFO << "IK failed, loop:" << step << endl;
            VR_INFO << "pos error:" << getCurrentError(goal) << endl;
        }

        robot->setJointValues(rns, jvBest);
        return false;
    }

    float GazeIK::getMaxPosError()
    {
        return maxPosError;
    }

} // namespace VirtualRobot
