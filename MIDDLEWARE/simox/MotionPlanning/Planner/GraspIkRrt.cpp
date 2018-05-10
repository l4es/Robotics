
#include "GraspIkRrt.h"

#include "../CSpace/CSpaceNode.h"
#include "../CSpace/CSpaceTree.h"
#include "../CSpace/CSpacePath.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <time.h>


//#define LOCAL_DEBUG(a) {SABA_INFO << a;};
#define LOCAL_DEBUG(a)

using namespace std;
using namespace VirtualRobot;

namespace Saba
{

    GraspIkRrt::GraspIkRrt(CSpaceSampledPtr cspace, VirtualRobot::ManipulationObjectPtr object, VirtualRobot::IKSolverPtr ikSolver, VirtualRobot::GraspSetPtr graspSet, float probabSampleGoal)
        : BiRrt(cspace, Rrt::eConnect, Rrt::eConnect), object(object), ikSolver(ikSolver), graspSet(graspSet), sampleGoalProbab(probabSampleGoal)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");
        THROW_VR_EXCEPTION_IF(!ikSolver, "NULL ikSolver");
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL graspSet");
        THROW_VR_EXCEPTION_IF(!cspace, "NULL cspace");
        THROW_VR_EXCEPTION_IF(!cspace->getRobot(), "NULL robot");

        if (sampleGoalProbab < 0)
        {
            SABA_WARNING << "Low value for sample goal probability, setting probability to 0" << endl;
            sampleGoalProbab = 0.0f;
        }

        if (sampleGoalProbab >= 1.0f)
        {
            SABA_WARNING << "High value for sample goal probability, setting probability to 0.99" << endl;
            sampleGoalProbab = 0.99f;
        }

        goalValid = true;
        name = "IK-RRT Planner";
        rns = ikSolver->getRobotNodeSet();
        THROW_VR_EXCEPTION_IF(!rns, "NULL robotNodeSet in ikSolver?!");
        graspSetWorking = graspSet->clone();
    }

    GraspIkRrt::~GraspIkRrt()
    {
    }

    bool GraspIkRrt::searchNewGoal()
    {
        if (graspSetWorking->getSize() == 0)
        {
            return true;
        }

        VirtualRobot::GraspPtr grasp = ikSolver->sampleSolution(object, graspSet);

        if (grasp)
        {
            Eigen::VectorXf config;
            rns->getJointValues(config);

            if (graspNodeMapping.find(grasp) != graspNodeMapping.end())
            {
                LOCAL_DEBUG("Grasp " <<  grasp->getName() << " already added..." << endl);
            }
            else
            {
                // remove from working set
                graspSetWorking->removeGrasp(grasp);
                SABA_INFO << endl << "Found new IK Solution: " << grasp->getName() << endl;
                SABA_INFO << config << endl;

                if (checkGoalConfig(config))
                {
                    return addIKSolution(config, grasp);
                }
                else
                {
                    SABA_INFO << "IK solution in collision " << endl;
                }

            }
        }

        // no error
        return true;
    }

    bool GraspIkRrt::addIKSolution(Eigen::VectorXf& config, VirtualRobot::GraspPtr grasp)
    {
        ikSolutions.push_back(config);

        if (tree2->getNrOfNodes() > 0)
        {
            if (connectComplete(config, tree2, lastAddedID2) == eSuccess)
            {
                LOCAL_DEBUG("ADDIK success" << endl);
                CSpaceNodePtr lastNode = tree2->getNode(lastAddedID2);
                THROW_VR_EXCEPTION_IF(!lastNode, "Internal error, expecting node here?!");
                graspNodeMapping[grasp] = lastNode;
                return true;
            }
            else
            {
                LOCAL_DEBUG("ADDIK failed" << endl);
            }
        }

        CSpaceNodePtr lastNode = tree2->appendNode(config, -(int)(ikSolutions.size()));
        THROW_VR_EXCEPTION_IF(!lastNode, "Internal error, expecting node here?!");
        graspNodeMapping[grasp] = lastNode;

        return true;
    }

    bool GraspIkRrt::checkGoalConfig(Eigen::VectorXf& config)
    {
        return cspace->isConfigValid(config);
    }

    bool GraspIkRrt::doPlanningCycle()
    {
        static const float rMult = (float)(1.0 / (double)(RAND_MAX));
        ExtensionResult resA, resB;
        float r = (float)rand() * rMult;

        if (r <= sampleGoalProbab || tree2->getNrOfNodes() == 0)
        {
            return searchNewGoal();
        }
        else
        {
            // do a normal extend/connect
            cspace->getRandomConfig(tmpConfig, true);
            LOCAL_DEBUG("----------------------------------------------------" << endl);
            LOCAL_DEBUG("Random Conf:" << endl << tmpConfig << endl);
            resA = connectUntilCollision(tmpConfig, tree, lastAddedID);

            if (resA == eError)
            {
                stopSearch = true;
            }

            if (resA == ePartial || resA == eSuccess)
            {
                // update config
                LOCAL_DEBUG("Last ID A:" << lastAddedID << endl);
                CSpaceNodePtr n = tree->getNode(lastAddedID);
                tmpConfig = n->configuration;
                LOCAL_DEBUG("Tmp goal B:" << endl << tmpConfig << endl);

                resB = connectUntilCollision(tmpConfig, tree2, lastAddedID2);

                if (resB == eError)
                {
                    stopSearch = true;
                }

                LOCAL_DEBUG("ExtResultB:" << resB << endl);
                LOCAL_DEBUG("Last ID B:" << lastAddedID2 << endl);

                if (resB == eSuccess)
                {
                    goalNode = tree2->getNode(lastAddedID2);

                    if (!goalNode)
                    {
                        SABA_ERROR << " no node for ID: " << lastAddedID2 << endl;
                        stopSearch = true;
                    }
                    else
                    {
                        found = true;
                    }
                }
            }
        }

        return !stopSearch;
    }


    bool GraspIkRrt::plan(bool bQuiet)
    {

        if (!bQuiet)
        {
            SABA_INFO << "Starting GraspIkRrt planner" << std::endl;
        }

        if (!isInitialized())
        {
            SABA_ERROR << " planner: not initialized..." << std::endl;
            return false;
        }

        cycles = 0;

        int distChecksStart = cspace->performaceVars_distanceCheck;
        int colChecksStart = cspace->performaceVars_collisionCheck;

        found = false;
        stopSearch = false;

        clock_t startClock = clock();

        solution.reset();

        RobotPtr robot = cspace->getRobot();
        bool bVisStatus = true;
        bVisStatus = robot->getUpdateVisualizationStatus();
        robot->setUpdateVisualization(false);



        // PLANNING LOOP
        do
        {
            if (!doPlanningCycle())
            {
                stopSearch = true;
            }

            cycles++;
        }
        while (!stopSearch && cycles < maxCycles && !found);

        clock_t endClock = clock();

        long diffClock = (long)(((float)(endClock - startClock) / (float)CLOCKS_PER_SEC) * 1000.0);
        planningTime = (float)diffClock;

        if (!bQuiet)
        {
            SABA_INFO << "Needed " << diffClock << " ms of processor time." << std::endl;

            SABA_INFO << "Created " << tree->getNrOfNodes() << " + " << tree2->getNrOfNodes() << " = " << tree->getNrOfNodes() + tree2->getNrOfNodes() << " nodes." << std::endl;
            SABA_INFO << "Collision Checks: " << (cspace->performaceVars_collisionCheck - colChecksStart) << std::endl;
            SABA_INFO << "Distance Calculations: " << (cspace->performaceVars_distanceCheck - distChecksStart) << std::endl;
            SABA_INFO << "IK solutions: " << ikSolutions.size() << std::endl;

            int nColChecks = (cspace->performaceVars_collisionCheck - colChecksStart);

            if (diffClock > 0)
            {
                float fPerf = (float)nColChecks / (float)diffClock * 1000.0f;
                std::cout << "Performance: " << fPerf << " cps (collision-checks per second)." << std::endl;
            }
        }

        if (robot && bVisStatus)
        {
            robot->setUpdateVisualization(bVisStatus);
        }

        if (found)
        {
            if (!bQuiet)
            {
                SABA_INFO << "Found RRT solution with " << cycles << " cycles." << std::endl;
            }

            createSolution(bQuiet);

            return true;
        }

        // something went wrong...
        if (cycles >= maxCycles)
        {
            SABA_WARNING << " maxCycles exceeded..." << std::endl;
        }

        if (stopSearch)
        {
            SABA_WARNING << " search was stopped..." << std::endl;
        }

        return false;

    }


    bool GraspIkRrt::setGoal(const Eigen::VectorXf& c)
    {
        THROW_VR_EXCEPTION("Not allowed here, goal configurations are sampled during planning..");
        return false;
    }

    void GraspIkRrt::reset()
    {
        BiRrt::reset();
        goalValid = true;
        graspSetWorking = graspSet->clone();
        ikSolutions.clear();
        graspNodeMapping.clear();
    }

    void GraspIkRrt::printConfig(bool printOnlyParams)
    {
        if (!printOnlyParams)
        {
            std::cout << "-- GraspIkRrt config --" << std::endl;
            std::cout << "------------------------------" << std::endl;
        }

        BiRrt::printConfig(true);

        if (!printOnlyParams)
        {
            std::cout << "------------------------------" << std::endl;
        }
    }


} // namespace
