
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "CSpaceTree.h"
#include "CSpaceNode.h"
#include "CSpacePath.h"
#include "CSpace.h"
#include "VirtualRobot/Robot.h"
#include "VirtualRobot/RobotNodeSet.h"
#include "float.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <time.h>

using namespace std;

//#define DO_THE_TESTS

namespace Saba
{

    CSpaceTree::CSpaceTree(CSpacePtr cspace)
    {
        if (!cspace)
        {
            THROW_SABA_EXCEPTION("NULL data, aborting...");
        }

        this->cspace = cspace;
        randMult = (float)(1.0 / (double)(RAND_MAX));
        updateChildren = false;

        dimension = cspace->getDimension();

        if (dimension < 1)
        {
            std::cout << "CSpaceTree: Initialization fails: INVALID DIMENSION" << std::endl;
            return;
        }

        tmpConfig.setZero(dimension);
    }

    CSpaceTree::~CSpaceTree()
    {

    }

    void CSpaceTree::lock()
    {
        mutex.lock();
    }

    void CSpaceTree::unlock()
    {
        mutex.unlock();
    }



    void CSpaceTree::reset()
    {
        idNodeMapping.clear();
        nodes.clear();
    }

    CSpaceNodePtr CSpaceTree::getNode(unsigned int id)
    {
        if (idNodeMapping.find(id) == idNodeMapping.end())
        {
            SABA_WARNING << " wrong ID: " << id << std::endl;
            return CSpaceNodePtr();
        }

        return idNodeMapping[id];
    }

    bool CSpaceTree::hasNode(CSpaceNodePtr n)
    {
        if (!n)
        {
            return false;
        }

        if (idNodeMapping.find(n->ID) == idNodeMapping.end())
        {
            return false;
        }

        return true;
    }

    float CSpaceTree::getPathDist(unsigned int idStart, unsigned int idEnd, bool useMetricWeights)
    {
        float result = 0.0f;

        if (!getNode(idStart) || !getNode(idStart))
        {
            SABA_ERROR << "CSpaceTree::getPathDist: start or goal id not correct..." << endl;
            return -1.0f;
        }

        unsigned int actID = idEnd;
        unsigned int actID2 = getNode(idEnd)->parentID;
        CSpaceNodePtr actNode;
        CSpaceNodePtr actNode2;

        while (actID2 != idStart)
        {
            actNode = getNode(actID);
            actNode2 = getNode(actID2);

            if (actID2 < 0 || actNode->parentID < 0 || !actNode || !actNode2)
            {
                std::cout << "CSpaceTree::getPathDist: error, no path from start to end ?!" << std::endl;
                return result;
            }

            result += cspace->calcDist(actNode->configuration, actNode2->configuration);

            actID = actID2;
            actID2 = actNode2->parentID;
        }

        // add last part
        result += cspace->calcDist(getNode(actID)->configuration, getNode(actID2)->configuration);

        return result;
    }

    // creates a copy of configuration
    CSpaceNodePtr CSpaceTree::appendNode(const Eigen::VectorXf& config, int parentID, bool calcDistance)
    {
        SABA_ASSERT(config.rows() == dimension)

        // create a new node with config, store it in nodeList and set parentID
        CSpaceNodePtr newNode = cspace->createNewNode();

        if (!newNode)
        {
            return newNode;
        }

        nodes.push_back(newNode);
        idNodeMapping[newNode->ID] = newNode;

        // parent of new CSpaceNode
        newNode->parentID = parentID;



        if (updateChildren && parentID >= 0)
        {
            CSpaceNodePtr n = getNode(parentID);

            if (!n)
            {
                SABA_ERROR << "CSpaceTree::appendNode: No parent node with id : " << parentID << std::endl;
            }
            else
            {
                n->children.push_back(newNode);
            }
        }

        // copy values
        newNode->configuration = config;
        // no distance information
        newNode->obstacleDistance = -1.0f;

        // calculate distances
        if (calcDistance)
        {
            float d = cspace->calculateObstacleDistance(newNode->configuration);
            newNode->obstacleDistance = d;
        }

#ifdef DO_THE_TESTS

        if (!cspace->IsConfigValid(config))
        {
            std::cout << "Appending not valid node!" << std::endl;
        }

#endif
        return newNode;
    }


    // returns true if full path was added
    bool CSpaceTree::appendPathUntilCollision(CSpaceNodePtr startNode, const Eigen::VectorXf& config, int* storeLastAddedID)
    {
        SABA_ASSERT(hasNode(startNode))
        SABA_ASSERT(config.rows() == dimension)

        float dist;
        CSpacePathPtr res = cspace->createPathUntilInvalid(startNode->configuration, config, dist);

        if (res->getNrOfPoints() <= 1)
        {
            return false;
        }

        res->erasePosition(0);
        bool appOK = appendPath(startNode, res, storeLastAddedID);

        if (!appOK)
        {
            return false;
        }

        return (dist == 1.0f);
    }

    bool CSpaceTree::appendPath(CSpaceNodePtr startNode, CSpacePathPtr path, int* storeLastAddedID)
    {
        SABA_ASSERT(hasNode(startNode))
        SABA_ASSERT(path)
        CSpaceNodePtr n = startNode;
        const std::vector<Eigen::VectorXf > data = path->getData();
        std::vector<Eigen::VectorXf >::const_iterator it;

        for (it = data.begin(); it != data.end(); it++)
        {
            n = appendNode(*it, n->ID);
        }

        if (storeLastAddedID)
        {
            *storeLastAddedID = n->ID;
        }

        return true;
    }

    // appends path from startNode to config, creates in-between nodes according to cspace if needed
    // no checks (for valid configs)
    // creates a copy of configuration
    bool CSpaceTree::appendPath(CSpaceNodePtr startNode, const Eigen::VectorXf& config, int* storeLastAddedID)
    {
        SABA_ASSERT(hasNode(startNode))
        SABA_ASSERT(config.rows() == dimension)

        CSpacePathPtr res = cspace->createPath(startNode->configuration, config);

        if (res->getNrOfPoints() <= 1)
        {
            return false;
        }

        res->erasePosition(0);
        return appendPath(startNode, res, storeLastAddedID);
    }


    void CSpaceTree::removeNode(CSpaceNodePtr n)
    {
        if (!n)
        {
            SABA_ERROR << ": NULL node" << std::endl;
            return;
        }

        if (n->ID < 0)
        {
            SABA_ERROR << ": wrong ID" << std::endl;
            return;
        }

        if (nodes.size() == 0)
        {
            SABA_ERROR << ": no nodes in tree" << std::endl;
            return;
        }

        vector<CSpaceNodePtr>::iterator it = nodes.begin();

        while (*it != n)
        {
            it++;

            if (it == nodes.end())
            {
                cout << "CSpaceTree::removeNode: node not in node vector ??? " << endl;
                return;
            }
        }

        nodes.erase(it);
        idNodeMapping.erase(n->ID);
        cspace->removeNode(n);
    }


    CSpaceNodePtr CSpaceTree::getNearestNeighbor(const Eigen::VectorXf& config, float* storeDist)
    {
        return getNode(getNearestNeighborID(config, storeDist));
    }


    unsigned int CSpaceTree::getNearestNeighborID(const Eigen::VectorXf& config, float* storeDist)
    {
        // goes through complete list of nodes, toDO: everything is better than this: eg octrees,something hierarchical...
        // update: only if we have more than 1000 nodes in the tree!

        if (nodes.size() == 0 || !cspace)
        {
            SABA_WARNING << "no nodes in tree..." << endl;
            return 0;
        }

        unsigned int bestID = nodes[0]->ID;
        float dist2 = cspace->calcDist2(config, nodes[0]->configuration, true);
        float test;

        for (unsigned int i = 1; i < nodes.size(); i++)
        {
            test = cspace->calcDist2(config, nodes[i]->configuration, true);

            if (test < dist2)
            {
                dist2 = test;
                bestID = nodes[i]->ID;
            }
        }

        if (storeDist != NULL)
        {
            *storeDist = sqrtf(dist2);
        }

        return bestID;
    }





    bool CSpaceTree::saveAllNodes(char const* filename)
    {
        std::ofstream file(filename, std::ios::out);

        if (!file.is_open())
        {
            std::cout << "ERROR: rrtCSpace::saveAllNodes: Could not open file to write." << std::endl;
            return false;
        }

        file << "# DO NOT EDIT LIST OF NODES #" << std::endl;
        file << "# FIRST ROW: KINEMATIC CHAIN NAME" << std::endl;
        file << "# SECOND ROW: DIMENSION" << std::endl;
        file << "# THIRD ROW: NUMBER OF ALL NODES" << std::endl;
        file << "# THEN ACTUAL DATA IN FOLLOWING FORMAT: ID CONFIG PARENTID" << std::endl;

        // save kinematic chain name used
        file << cspace->getRobotNodeSet()->getName() << std::endl;

        // save rrt dimension used for planning
        file << dimension << std::endl;

        // save number of nodes
        file << nodes.size() << std::endl;

        CSpaceNodePtr actualNode;

        for (unsigned int i = 0; i < nodes.size(); i++)
        {
            // save ID, configuration and parentID of each node in one row
            actualNode = nodes[i];
            file << actualNode->ID << " ";

            for (unsigned int j = 0; j < dimension; j++)
            {
                file << actualNode->configuration[j] << " ";
            }

            file << actualNode->parentID << std::endl;
        }

        file.close();
        return true;
    }

    CSpaceNodePtr CSpaceTree::getLastAddedNode()
    {
        if (nodes.size() == 0)
        {
            return CSpaceNodePtr();
        }

        return nodes[nodes.size() - 1];
    }


    bool CSpaceTree::createPath(CSpaceNodePtr startNode, CSpaceNodePtr goalNode, CSpacePathPtr fillPath)
    {
        // creates a path from goal to startNode

        if (!goalNode || !startNode || !fillPath)
        {
            return false;
        }

        std::vector<int> tmpSol;
        fillPath->reset();
        int actID = goalNode->ID;
        CSpaceNodePtr actNode = goalNode;
        bool found = false;

        while (actID >= 0 && actNode && !found)
        {
            actNode = getNode(actID);

            if (actNode)
            {
                tmpSol.push_back(actID);

                //fillPath->addPoint(actNode->configuration);
                if (actID == startNode->ID)
                {
                    found = true;
                }
                else
                {
                    actID = actNode->parentID;
                }
            }
        }

        if (!found)
        {
            std::cout << "CSpaceTree::createPath: Start node not a parent of goalNode... goalID:" << goalNode->ID << ", startID:" << startNode->ID << std::endl;
        }
        else
        {
            // revert path
            for (int i = (int)tmpSol.size() - 1; i >= 0; i--)
            {
                fillPath->addPoint(getNode(tmpSol[i])->configuration);
            }
        }

        return found;
    }

    /*
    void CSpaceTree::printNode(CSpaceNodePtr n)
    {
        if (!n)
            return;
        cspace->printConfig(n->configuration);
    }*/

    /*
    void CSpaceTree::printNode(unsigned int id)
    {
        CSpaceNodePtr n = getNode(id);
        cspace->printConfig(n->configuration);
    }
    */

    float CSpaceTree::getTreeLength(bool useMetricWeights)
    {
        unsigned int nMaxNodes = (int)nodes.size();
        float res = 0;

        for (unsigned int i = 0; i < nMaxNodes; i++)
        {
            if (nodes[i] && nodes[i]->parentID >= 0 && nodes[i]->parentID < (int)nMaxNodes)
            {
                res += cspace->calcDist(nodes[i]->configuration, nodes[nodes[i]->parentID]->configuration, !useMetricWeights);
            }
        }

        return res;
    }

    unsigned int CSpaceTree::getNrOfNodes() const
    {
        return nodes.size();
    }

    std::vector<CSpaceNodePtr> CSpaceTree::getNodes()
    {
        return nodes;
    }

    unsigned int CSpaceTree::getDimension() const
    {
        return dimension;
    }

    Saba::CSpacePtr CSpaceTree::getCSpace()
    {
        return cspace;
    }

} // namespace Saba
