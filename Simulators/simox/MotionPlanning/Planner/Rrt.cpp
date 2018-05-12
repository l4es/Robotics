
#include "Rrt.h"
#include "../CSpace/CSpaceNode.h"
#include "../CSpace/CSpaceTree.h"
#include "../CSpace/CSpacePath.h"
#include "VirtualRobot/Robot.h"
#include <time.h>


using namespace std;
using namespace VirtualRobot;

namespace Saba
{

    Rrt::Rrt(CSpaceSampledPtr cspace, RrtMethod mode, float probabilityExtendToGoal)
        : MotionPlanner(boost::dynamic_pointer_cast<CSpace>(cspace))
    {
        rrtMode = mode;
        tree.reset(new CSpaceTree(cspace));

        if (probabilityExtendToGoal <= 0)
        {
            SABA_ERROR << " probabilityExtendToGoal is wrong: " << probabilityExtendToGoal << endl;
            probabilityExtendToGoal = 0.1f;
        }

        if (probabilityExtendToGoal >= 1.0f)
        {
            SABA_ERROR << " probabilityExtendToGoal is wrong: " << probabilityExtendToGoal << endl;
            probabilityExtendToGoal = 0.9f;
        }

        extendGoToGoal = probabilityExtendToGoal;

        this->extendStepSize = cspace->getSamplingSize();
        tmpConfig.setZero(dimension);
        lastAddedID = -1;
    }

    Rrt::~Rrt()
    {
    }


    bool Rrt::plan(bool bQuiet)
    {
        if (!bQuiet)
        {
            SABA_INFO << "Starting Rrt planner" << std::endl;

            switch (rrtMode)
            {
                case eExtend:
                    cout << "-- Mode: RRT-EXTEND" << endl;
                    break;

                case eConnect:
                    cout << "-- Mode: RRT-CONNECT" << endl;
                    break;

                case eConnectCompletePath:
                    cout << "-- Mode: RRT-CONNECT (only complete paths)" << endl;
                    break;

                default:
                    break;
            }
        }

        float rand_mult = (float)(1.0 / (double)(RAND_MAX));
        float r;

        if (!isInitialized())
        {
            SABA_ERROR << " planner: not initialized..." << std::endl;
            return false;
        }

        cycles = 0;
        int distChecksStart = cspace->performaceVars_distanceCheck;
        int colChecksStart = cspace->performaceVars_collisionCheck;

        bool found = false;
        stopSearch = false;

        clock_t startClock = clock();

        solution.reset();

        RobotPtr robot = cspace->getRobot();
        bool bVisStatus = true;

        if (robot)
        {
            bVisStatus = robot->getUpdateVisualizationStatus();
            robot->setUpdateVisualization(false);
        }

        ExtensionResult extResult;

        // the extension loop
        do
        {
            // CHOOSE A RANDOM CONFIGURATION
            // check if we want to go to the goal directly or extend randomly
            r = (float)rand() * rand_mult;

            if (r <= extendGoToGoal)
            {
                switch (rrtMode)
                {
                    case eExtend:
                        extResult = extend(goalConfig, tree, lastAddedID);
                        break;

                    case eConnect:
                        extResult = connectUntilCollision(goalConfig, tree, lastAddedID);
                        break;

                    case eConnectCompletePath:
                        extResult = connectComplete(goalConfig, tree, lastAddedID);
                        break;

                    default:
                        break;
                }

                if (extResult == eSuccess)
                {

                    goalNode = cspace->getNode(lastAddedID);

                    if (!goalNode)
                    {
                        SABA_ERROR << " no node for ID: " << lastAddedID << endl;
                        stopSearch = true;
                    }
                    else
                    {
                        if (!goalConfig.isApprox(goalNode->configuration))
                        {
                            SABA_WARNING << "GoalConfig: (" << goalConfig << ") != goalNode (" << goalNode->configuration << ")" << endl;
                        }

                        //SABA_ASSERT(goalConfig.isApprox(goalNode->configuration));
                        found = true;
                    }
                }
                else if (extResult == eError)
                {
                    stopSearch = true;
                }
            }
            else
            {
                // extend randomly, create a random position in config space
                cspace->getRandomConfig(tmpConfig);

                switch (rrtMode)
                {
                    case eExtend:
                        extResult = extend(tmpConfig, tree, lastAddedID);
                        break;

                    case eConnect:
                        extResult = connectUntilCollision(tmpConfig, tree, lastAddedID);
                        break;

                    case eConnectCompletePath:
                        extResult = connectComplete(tmpConfig, tree, lastAddedID);
                        break;

                    default:
                        break;
                }

                if (extResult == eError)
                {
                    stopSearch = true;
                }
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

            SABA_INFO << "Created " << tree->getNrOfNodes() << " nodes." << std::endl;
            SABA_INFO << "Collision Checks: " << (cspace->performaceVars_collisionCheck - colChecksStart) << std::endl;
            SABA_INFO << "Distance Calculations: " << (cspace->performaceVars_distanceCheck - distChecksStart) << std::endl;

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

    Rrt::ExtensionResult Rrt::extend(Eigen::VectorXf& c, CSpaceTreePtr tree, int& storeLastAddedID)
    {
        // NEAREST NEIGHBOR OF RANDOM CONFIGURATION
        CSpaceNodePtr nn = tree->getNearestNeighbor(c);

        SABA_ASSERT(nn);

        CSpacePtr cspace = tree->getCSpace();

        // length of the new extension step
        float totalLength = cspace->calcDist(nn->configuration, c);

        bool reached = false;

        // set randomly found configuration if distance is smaller than specific length
        if (totalLength <= extendStepSize)
        {
            tmpConfig = c;
            reached = true;
        }
        else
        {
            float factor = extendStepSize / totalLength;
            // go a specific length in the direction of randomly found configuration
            tmpConfig = nn->configuration + ((c - nn->configuration) * factor);
            //for (unsigned int i = 0; i < m_nDimension; i++)
            //  m_pExtendRrtNewValue[i] = m_nnNode->configuration[i] + ((extGoal[i] - m_nnNode->configuration[i]) * factor);
        }

        // CHECK PATH FOR COLLISIONS AND VALID NODES
        if (cspace->isPathValid(nn->configuration, tmpConfig))
        {
            // ADD IT TO RRT TREE
            if (!tree->appendPath(nn, tmpConfig, &storeLastAddedID))
            {
                return eError;
            }

            if (reached)
            {
                return eSuccess;
            }
            else
            {
                return ePartial; // ADVANCED
            }
        }//PATH NOT COLLISION FREE

        return eFailed; // EXTEND FAILS
    }


    Rrt::ExtensionResult Rrt::connectComplete(Eigen::VectorXf& c, CSpaceTreePtr tree, int& storeLastAddedID)
    {
        // NEAREST NEIGHBOR OF RANDOM CONFIGURATION
        CSpaceNodePtr nn = tree->getNearestNeighbor(c);

        SABA_ASSERT(nn);

        // CHECK PATH FOR COLLISIONS AND VALID NODES
        if (cspace->isPathValid(nn->configuration, c))
        {
            // ADD IT TO RRT TREE
            if (!tree->appendPath(nn, c, &storeLastAddedID))
            {
                return eError;
            }

            return eSuccess; // REACHED
        }//PATH NOT COLLISION FREE

        return eFailed; // CONNECT FAILS
    }

    Rrt::ExtensionResult Rrt::connectUntilCollision(Eigen::VectorXf& c, CSpaceTreePtr tree, int& storeLastAddedID)
    {
        // NEAREST NEIGHBOR OF RANDOM CONFIGURATION
        CSpaceNodePtr nn = tree->getNearestNeighbor(c);

        int oldLastID = storeLastAddedID;

        // CHECK PATH FOR COLLISIONS AND VALID NODES
        if (tree->appendPathUntilCollision(nn, c, &storeLastAddedID))
        {
            return eSuccess; // REACHED (add m_pExtendRrtRandValue to rrt tree)
        }//PATH NOT COLLISION FREE

        if (oldLastID == storeLastAddedID)
        {
            return eFailed;    // CONNECT FAILS
        }
        else
        {
            return ePartial;    // added some new nodes
        }
    }

    void Rrt::printConfig(bool printOnlyParams)
    {
        if (!printOnlyParams)
        {
            std::cout << "-- Rrt config --" << std::endl;
            std::cout << "------------------------------" << std::endl;
        }

        float DCDsize = 0.0f;
        float Pathsize = 0.0f;
        CSpaceSampledPtr cs = boost::dynamic_pointer_cast<CSpaceSampled>(cspace);

        if (cs)
        {
            DCDsize = cs->getSamplingSizeDCD();
            Pathsize = cs->getSamplingSize();
        }

        std::cout << "-- RRT-Extend step size: " << extendStepSize << std::endl;
        std::cout << "-- C-Space: Add new Paths sampling size: " << Pathsize << std::endl;
        std::cout << "-- C-Space: DCD sampling size: " << DCDsize << std::endl;
        std::cout << "-- Probability: extend to goal: " << extendGoToGoal << std::endl;

        MotionPlanner::printConfig(true);

        switch (rrtMode)
        {
            case eExtend:
                cout << "-- Mode: RRT-EXTEND" << endl;
                break;

            case eConnect:
                cout << "-- Mode: RRT-CONNECT" << endl;
                break;

            case eConnectCompletePath:
                cout << "-- Mode: RRT-CONNECT (only complete paths)" << endl;
                break;

            default:
                break;
        }

        if (!printOnlyParams)
        {
            std::cout << "------------------------------" << std::endl;
        }
    }


    bool Rrt::setStart(const Eigen::VectorXf& c)
    {
        if (!MotionPlanner::setStart(c))
        {
            return false;
        }

        if (tree->getNrOfNodes() > 0)
        {
            SABA_WARNING << "Removing all nodes from tree..." << endl;
            tree->reset();
        }

        // init root node of RRT
        startNode = tree->appendNode(c, -1);

        return true;
    }


    bool Rrt::setGoal(const Eigen::VectorXf& c)
    {
        if (!MotionPlanner::setGoal(c))
        {
            return false;
        }

        return true;
    }

    // uses last added CSpaceNode as start
    bool Rrt::createSolution(bool bQuiet)
    {

        // check if we have a path

        if (!goalNode || goalNode->parentID < 0)
        {
            SABA_WARNING << " no path to goal..." << std::endl;
            return false;
        }

        std::vector<int> tmpSol;

        int count = 0;
        CSpaceNodePtr actNode;
        int nextID;
        bool failure = false;

        // check if last node has goal configuration
        CSpaceNodePtr lastNode = tree->getNode(goalNode->ID);//parentID);
        tmpSol.push_back(lastNode->ID);
        actNode = lastNode;

        // go through all solution nodes backwards
        do
        {
            nextID = actNode->parentID;

            if (nextID < 0)
            {
                if (actNode->ID != startNode->ID)
                {
                    SABA_WARNING << " oops something went wrong..." << std::endl;
                    failure = true;
                }
            }
            else
            {
                // got to parent CSpaceNode
                actNode = tree->getNode(nextID);
                tmpSol.push_back(nextID);
            }

            count++;

            if (!actNode)
            {
                failure = true;
            }
        }
        while (!failure && actNode->ID != startNode->ID);

        if (!failure)
        {
            solution.reset(new CSpacePath(cspace));

            // store solution in correct order
            for (int i = (int)tmpSol.size() - 1; i >= 0; i--)
            {
                solution->addPoint(tree->getNode(tmpSol[i])->configuration);
            }

            if (!bQuiet)
            {
                SABA_INFO << "Created solution with " << solution->getNrOfPoints() << " nodes." << std::endl;
            }

            return true;
        }
        else
        {
            return false;
        }
    }

    void Rrt::reset()
    {
        MotionPlanner::reset();
        startNode.reset();
        goalNode.reset();

        if (tree)
        {
            tree->reset();
        }
    }



    void Rrt::setProbabilityExtendToGoal(float p)
    {
        extendGoToGoal = p;

        if (extendGoToGoal <= 0)
        {
            extendGoToGoal = 0.1f;
        }

        if (extendGoToGoal >= 1.0f)
        {
            extendGoToGoal = 0.9f;
        }
    }

    Saba::CSpaceTreePtr Rrt::getTree()
    {
        return tree;
    }

} // namespace
