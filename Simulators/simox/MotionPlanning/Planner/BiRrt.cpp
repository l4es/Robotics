
#include "BiRrt.h"

#include "../CSpace/CSpaceNode.h"
#include "../CSpace/CSpaceTree.h"
#include "../CSpace/CSpacePath.h"
#include "VirtualRobot/Robot.h"
#include <time.h>


//#define LOCAL_DEBUG(a) {SABA_INFO << a;};
#define LOCAL_DEBUG(a)

using namespace std;
using namespace VirtualRobot;

namespace Saba
{

    BiRrt::BiRrt(CSpaceSampledPtr cspace, RrtMethod modeA, RrtMethod modeB)
        : Rrt(cspace, modeA)
    {
        rrtMode2 = modeB;
        tree2.reset(new CSpaceTree(cspace));
        lastAddedID2 = -1;
    }

    BiRrt::~BiRrt()
    {
    }


    bool BiRrt::plan(bool bQuiet)
    {

        if (!bQuiet)
        {
            SABA_INFO << "Starting BiRrt planner" << std::endl;

            switch (rrtMode)
            {
                case eExtend:
                    cout << "-- ModeA: RRT-EXTEND" << endl;
                    break;

                case eConnect:
                    cout << "-- ModeA: RRT-CONNECT" << endl;
                    break;

                case eConnectCompletePath:
                    cout << "-- ModeA: RRT-CONNECT (only complete paths)" << endl;
                    break;

                default:
                    break;
            }

            switch (rrtMode2)
            {
                case eExtend:
                    cout << "-- ModeB: RRT-EXTEND" << endl;
                    break;

                case eConnect:
                    cout << "-- ModeB: RRT-CONNECT" << endl;
                    break;

                case eConnectCompletePath:
                    cout << "-- ModeB: RRT-CONNECT (only complete paths)" << endl;
                    break;

                default:
                    break;
            }
        }

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

        ExtensionResult extResultA;
        ExtensionResult extResultB;

        bool switched = false;
        int* lastIDA = &lastAddedID;
        int* lastIDB = &lastAddedID2;
        CSpaceTreePtr treeA = tree;
        CSpaceTreePtr treeB = tree2;
        RrtMethod rrtModeA = rrtMode;
        RrtMethod rrtModeB = rrtMode2;

        // PLANNING LOOP
        do
        {
            // CHOOSE A RANDOM CONFIGURATION (NOT GOAL DIRECTED, ONLY RANDOMLY)
            cspace->getRandomConfig(tmpConfig, true);
            LOCAL_DEBUG("----------------------------------------------------" << endl);
            LOCAL_DEBUG("Random Conf:" << endl << tmpConfig << endl);

            if (switched)
            {
                LOCAL_DEBUG("SWITCHED" << endl);
                lastIDA = &lastAddedID2;
                lastIDB = &lastAddedID;
                treeA = tree2;
                treeB = tree;
                rrtModeA = rrtMode2;
                rrtModeB = rrtMode;
            }
            else
            {
                lastIDA = &lastAddedID;
                lastIDB = &lastAddedID2;
                treeA = tree;
                treeB = tree2;
                rrtModeA = rrtMode;
                rrtModeB = rrtMode2;
            }

            switch (rrtModeA)
            {
                case eExtend:
                    extResultA = extend(tmpConfig, treeA, *lastIDA);
                    break;

                case eConnect:
                    extResultA = connectUntilCollision(tmpConfig, treeA, *lastIDA);
                    break;

                case eConnectCompletePath:
                    extResultA = connectComplete(tmpConfig, treeA, *lastIDA);
                    break;

                default:
                    break;
            }

            LOCAL_DEBUG("ExtResultA:" << extResultA << endl);

            if (extResultA == eError)
            {
                stopSearch = true;
            }

            if (extResultA == ePartial || extResultA == eSuccess)
            {
                // update config
                LOCAL_DEBUG("Last ID A:" << *lastIDA << endl);
                CSpaceNodePtr n = treeA->getNode(*lastIDA);
                tmpConfig = n->configuration;
                LOCAL_DEBUG("Tmp goal B:" << endl << tmpConfig << endl);

                switch (rrtModeB)
                {
                    case eExtend:
                        extResultB = extend(tmpConfig, treeB, *lastIDB);
                        break;

                    case eConnect:
                        extResultB = connectUntilCollision(tmpConfig, treeB, *lastIDB);
                        break;

                    case eConnectCompletePath:
                        extResultB = connectComplete(tmpConfig, treeB, *lastIDB);
                        break;

                    default:
                        break;
                }

                LOCAL_DEBUG("Last ID B:" << *lastIDB << endl);
                LOCAL_DEBUG("ExtResultB:" << extResultB << endl);

                if (extResultB == eError)
                {
                    stopSearch = true;
                }

                if (extResultB == eSuccess)
                {
                    goalNode = treeB->getNode(*lastIDB);

                    if (!goalNode)
                    {
                        SABA_ERROR << " no node for ID: " << lastIDB << endl;
                        stopSearch = true;
                    }
                    else
                    {
                        found = true;
                    }
                }
            }


            cycles++;
            switched = !switched;

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


    // uses lastAddedID and lastAddedID2
    bool BiRrt::createSolution(bool bQuiet)
    {
        // delete an existing solution if necessary
        solution.reset();

        // vectors to save the node ids for solution path
        std::vector<int> tmpSol; // tree
        std::vector<int> tmpSol2; // tree2


        // bridgeover node!!!
        CSpaceNodePtr actNode = tree->getNode(lastAddedID);
        CSpaceNodePtr actNode2 = tree2->getNode(lastAddedID2);

        if (!bQuiet)
        {
            SABA_INFO << "node1:" << lastAddedID << ", node2: " << lastAddedID2 << std::endl;
        }

        if (!actNode || !actNode2)
        {
            SABA_ERROR << "No nodes for Solution ?!" << std::endl;
            return false;
        }

        // bridgeover node only once in solution vector
        tmpSol.push_back(actNode->ID);

        int nextID;
        int nextID2;

        int count1 = 0;
        int count2 = 0;
        // go through all solution nodes backwards in tree till m_pStartNode (parentID = -1)
        nextID = actNode->parentID;

        while (nextID >= 0)
        {
            count1++;
            actNode = tree->getNode(nextID);
            tmpSol.push_back(nextID);
            nextID = actNode->parentID;
        }

        // go through all solution nodes backwards in tree2 till m_pGoalNode (parentID = -1)
        nextID2 = actNode2->parentID;

        while (nextID2 >= 0)
        {
            count2++;
            actNode2 = tree2->getNode(nextID2);
            tmpSol2.push_back(nextID2);
            nextID2 = actNode2->parentID;
        }

        if (!bQuiet)
        {
            SABA_INFO << "Count1: " << count1 << " - count2: " << count2 << std::endl;
        }

        //reverse the solution vector from bridgeover node to start node (correct order)
        reverse(tmpSol.begin(), tmpSol.end());

        // Create the CSpacePath, first start node till bridgeover node, then bridgeover node till goal node
        solution.reset(new CSpacePath(cspace));

        for (unsigned int i = 0; i < tmpSol.size(); i++)
        {
            solution->addPoint(tree->getNode(tmpSol[i])->configuration);
        }

        for (unsigned int i = 0; i < tmpSol2.size(); i++)
        {
            solution->addPoint(tree2->getNode(tmpSol2[i])->configuration);
        }

        if (!bQuiet)
        {
            SABA_INFO << "Created solution with " << solution->getNrOfPoints() << " nodes." << std::endl;
        }

        //solution->checkDistance(tree->getColCheckSamplingSize());
        return true;
    }

    bool BiRrt::setStart(const Eigen::VectorXf& c)
    {
        return Rrt::setStart(c);
    }

    bool BiRrt::setGoal(const Eigen::VectorXf& c)
    {
        if (!Rrt::setGoal(c))
        {
            return false;
        }

        // copy goal values
        if (goalNode)
        {
            tree2->removeNode(goalNode);
        }

        goalNode =  tree2->appendNode(c, -1);

        return true;
    }

    void BiRrt::reset()
    {
        Rrt::reset();

        if (tree2)
        {
            tree2->reset();
        }
    }

    void BiRrt::printConfig(bool printOnlyParams)
    {
        if (!printOnlyParams)
        {
            std::cout << "-- BiRrt config --" << std::endl;
            std::cout << "------------------------------" << std::endl;
        }

        Rrt::printConfig(true);

        switch (rrtMode2)
        {
            case eExtend:
                cout << "-- Mode2: RRT-EXTEND" << endl;
                break;

            case eConnect:
                cout << "-- Mode2: RRT-CONNECT" << endl;
                break;

            case eConnectCompletePath:
                cout << "-- Mode2: RRT-CONNECT (only complete paths)" << endl;
                break;

            default:
                break;
        }

        if (!printOnlyParams)
        {
            std::cout << "------------------------------" << std::endl;
        }
    }

    CSpaceTreePtr BiRrt::getTree2()
    {
        return tree2;
    }

} // namespace
