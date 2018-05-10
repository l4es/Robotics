#include "CSpace.h"
#include "CSpaceNode.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include "Sampler.h"
#include "CSpaceTree.h"
#include "CSpacePath.h"
#include "ConfigurationConstraint.h"
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include "float.h"
#include <cmath>
#include <fstream>
#include <iomanip>
#include <time.h>

#define GET_RANDOM_DATA_FROM_64BIT_ADDRESS(a) (int)(0xFF & (long)a) | (0xFF00 & ((long)a >> 16))

using namespace std;

namespace Saba
{

    SABA_IMPORT_EXPORT boost::mutex CSpace::colCheckMutex;
    SABA_IMPORT_EXPORT int CSpace::cloneCounter = 0;

    //#define DO_THE_TESTS
    CSpace::CSpace(VirtualRobot::RobotPtr robot, VirtualRobot::CDManagerPtr collisionManager, VirtualRobot::RobotNodeSetPtr robotNodes, unsigned int maxConfigs, unsigned int randomSeed)
    {
        if (maxConfigs <= 0)
        {
            THROW_SABA_EXCEPTION("CSpaceTree: Initialization fails: INVALID MAX_CONFIGS");
        }

        performaceVars_collisionCheck = 0;
        performaceVars_distanceCheck = 0;
        checkForBorderlessDims = true;
        robo = robot;
        cdm = collisionManager;

        if (randomSeed == 0)
        {
#ifdef __LP64__
            // This is for machines with 64Bit addresses and 32Bit int datatype
            randomSeed = (unsigned int)(time(NULL) + GET_RANDOM_DATA_FROM_64BIT_ADDRESS(this)) % 10000;
#else
            randomSeed = (unsigned int)(time(NULL) + (int)this) % 10000;
#endif
        }

        this->randomSeed = randomSeed;

        randMult = (float)(1.0 / (double)(RAND_MAX));

        useMetricWeights = false;
        multiThreaded = false;
        stopPathCheck = false;

        sampleAlgorithm.reset();

        if (!robo)
        {
            THROW_SABA_EXCEPTION("CSpace: Initialization fails: NO ROBOT");
        }

        srand(randomSeed);
        //std::cout << "Using random seed: " << m_randomSeed << std::endl;

        dimension = 0;

        if (!robotNodes || robotNodes->getAllRobotNodes().size() == 0)
        {
            THROW_SABA_EXCEPTION("CSpace: Initialization fails: NO ROBOT JOINTS");
        }

        this->robotNodes = robotNodes;
        robotJoints = robotNodes->getAllRobotNodes();

        dimension = robotJoints.size();

        if (dimension < 1)
        {
            THROW_SABA_EXCEPTION("CSpace: Initialization fails: INVALID DIMENSION")
        }

        boundaryMin.setZero(dimension);
        boundaryMax.setZero(dimension);
        boundaryDist.setZero(dimension);

        for (unsigned int i = 0; i < dimension; i++)
        {
            boundaryMin[i] = robotJoints[i]->getJointLimitLo();
            boundaryMax[i] = robotJoints[i]->getJointLimitHi();
            boundaryDist[i] = fabs(boundaryMax[i] - boundaryMin[i]);

            if (boundaryDist[i] == 0)
            {
                SABA_WARNING << "Limits for joint " << robotJoints[i]->getName() << " not set correctly. Degenerated dimension (" << i << ") in C-Space." << endl;
            }
        }

        metricWeights.setZero(dimension);

        for (unsigned int i = 0; i < dimension; i++)
        {
            metricWeights[i] = 1.0;
        }

        checkForBorderlessDimensions(checkForBorderlessDims);

        SABA_INFO << " dimension: " << dimension << ", random Seed: " << randomSeed << endl;

        // allocate configs
        maxNodes = maxConfigs;

        for (int i = 0; i < maxNodes; i++)
        {
            CSpaceNodePtr node(new CSpaceNode());
            node->ID = nodes.size();
            node->parentID = -666;
            node->dynDomRadius = 0;
            node->obstacleDistance = -1.0f;
            node->allocated = false;
            freeNodes.push_back(node);
            nodes.push_back(node);
        }
    }


    CSpace::~CSpace()
    {
        freeNodes.clear();
        nodes.clear();
    }


    void CSpace::requestStop()
    {
        stopPathCheck = true;
    }

    void CSpace::reset()
    {
        resetPerformanceVars();
        unsigned int i;

        for (i = 0; i < nodes.size(); i++)
        {
            nodes[i]->allocated = false;
        }

        freeNodes = nodes;
    }

    CSpaceNodePtr CSpace::getNode(unsigned int id)
    {
        if (id >= nodes.size())
        {
            SABA_ERROR << "ID " << id << " not known....";
            return CSpaceNodePtr();
        }

        if (!nodes[id]->allocated)
        {
            SABA_WARNING << "ID " << id << " not allocated....";
        }

        return nodes[id];
    }

    bool CSpace::checkSolution(CSpacePathPtr solution, bool bVerbose)
    {
        if (!solution)
        {
            std::cout << "NULL solution..." << std::endl;
            return false;
        }

        unsigned int s = solution->getNrOfPoints();

        if (bVerbose)
        {
            SABA_INFO << "Checking " << s << " segments:" << endl;
        }

        for (unsigned int i = s - 1; i > 0; i--)
        {
            clock_t t1 = clock();
            bool bColFree = isPathValid(solution->getPoint(i), solution->getPoint(i - 1));
            clock_t t2 = clock();

            if (bVerbose && t2 - t1 > 0)
            {
                cout << "i:" << i << "->t:" << t2 - t1 << " , ";
            }

            if (!bColFree)
            {
                return false;
            }
        }

        if (bVerbose)
        {
            cout << endl;
        }

        return true;
    }

    bool CSpace::checkTree(CSpaceTreePtr tree)
    {
        if (!tree)
        {
            std::cout << "NULL tree..." << std::endl;
            return false;
        }

        std::vector<CSpaceNodePtr> vNodes = tree->getNodes();

        unsigned int s = (unsigned int)vNodes.size();

        for (unsigned int i = 0; i < s; i++)
        {
            CSpaceNodePtr pN1 = vNodes[i];

            if (pN1 && pN1->parentID >= 0 && tree->getNode(pN1->parentID))
            {
                CSpaceNodePtr pN2 = tree->getNode(pN1->parentID);

                if (!isPathValid((pN1->configuration), (pN2->configuration)))
                {
                    return false;
                }
            }
        }

        return true;
    }

    void CSpace::setBoundaries(const Eigen::VectorXf& min, const Eigen::VectorXf& max)
    {
        SABA_ASSERT(min.rows() == dimension)

        for (unsigned int i = 0; i < dimension; i++)
        {
            setBoundary(i, min[i], max[i]);
        }
    }


    void CSpace::setBoundary(unsigned int dim, float min, float max)
    {
        SABA_ASSERT(dim < dimension)

        if (min > max)
        {
            float t = min;
            min = max;
            max = t;
        }

        boundaryMin[dim] = min;
        boundaryMax[dim] = max;
        boundaryDist[dim] = fabs(max - min);
        checkForBorderlessDimensions(checkForBorderlessDims);
    }

    void CSpace::setMetricWeights(const Eigen::VectorXf& weights)
    {
        SABA_ASSERT(weights.rows() == dimension)

        for (unsigned int i = 0; i < dimension; i++)
        {
            metricWeights[i] = weights[i];
            //std::cout << "Dim: " << i << " Weight: " << metricWeights[i] << std::endl;
        }

        if (sampleAlgorithm)
        {
            sampleAlgorithm->enableMetricWeights(metricWeights);
        }

        enableWeights(true);
    }

    void CSpace::checkForBorderlessDimensions(bool enable)
    {
        checkForBorderlessDims = enable;
        borderLessDimension.resize(dimension);

        for (unsigned int i = 0; i < dimension; i++)
        {
            borderLessDimension[i] = enable ? isBorderlessDimension(i) : false;
        }
    }

    float CSpace::calcDist(const Eigen::VectorXf& c1, const Eigen::VectorXf& c2, bool forceDisablingMetricWeights)
    {
        return sqrtf(calcDist2(c1, c2, forceDisablingMetricWeights));
    }

    float CSpace::calcDist2(const Eigen::VectorXf& c1, const Eigen::VectorXf& c2, bool forceDisablingMetricWeights)
    {
        SABA_ASSERT(c1.rows() == dimension)
        SABA_ASSERT(c2.rows() == dimension)

        float res = 0.0f;
        float dist;

        for (unsigned int i = 0 ; i < dimension; i++)
        {
            dist = c1[i] - c2[i];

            if (borderLessDimension[i])
            {
                // borderless
                if (fabs(dist) > M_PI)
                {
                    dist = 2.0f * (float)M_PI - fabs(dist);
                }
            }

            if (useMetricWeights && !forceDisablingMetricWeights)
            {
                res += metricWeights[i] * metricWeights[i] * dist * dist;
            }
            else
            {
                res += dist * dist;
            }
        }


        return res;
    }


    void CSpace::generateNewConfig(const Eigen::VectorXf& randomConfig, const Eigen::VectorXf& nearestConfig, Eigen::VectorXf& storeNewConfig, float stepSize, float preCalculatedDist /*=-1.0*/)
    {
        SABA_ASSERT(randomConfig.rows() == dimension)
        SABA_ASSERT(nearestConfig.rows() == dimension)
        SABA_ASSERT(storeNewConfig.rows() == dimension)

        if (preCalculatedDist < 0)
        {
            preCalculatedDist = calcDist(randomConfig, nearestConfig);
        }

        // distance is smaller than extendStepSize: set randomly found configuration
        if (preCalculatedDist <= stepSize)
        {
            storeNewConfig = randomConfig;
            return;
        }

        // distance is greater than stepSize: go stepSize in direction of the randomly found configuration
        float factor = stepSize / preCalculatedDist;

        for (int i = 0; i < (int)dimension; i++)
        {
            storeNewConfig[i] = interpolate(nearestConfig, randomConfig, i, factor);
        }
    }

    void CSpace::getDirectionVector(const Eigen::VectorXf& c1, const Eigen::VectorXf& c2, Eigen::VectorXf& storeDir, float length)
    {
        SABA_ASSERT(c1.rows() == dimension)
        SABA_ASSERT(c2.rows() == dimension)
        SABA_ASSERT(storeDir.rows() == dimension)
        float l = calcDist(c1, c2);

        if (l == 0.0)
        {
            std::cout << "getDirectionVector:Identical vectors..." << std::endl;

            for (unsigned int i = 0; i < dimension; i++)
            {
                storeDir[i] = 0.1f;
            }

            return;
        }

        float factor = length / l;

        for (unsigned int i = 0; i < dimension; i++)
        {
            storeDir[i] = c1[i] - interpolate(c1, c2, i, factor);
        }
    }


    float CSpace::calculateObstacleDistance(const Eigen::VectorXf& config)
    {
        SABA_ASSERT(config.rows() == dimension)

        performaceVars_distanceCheck++;

        if (multiThreaded)
        {
            colCheckMutex.lock();
        }

        // set configuration (joint values)
        robo->setJointValues(robotNodes, config);
        float d = cdm->getDistance();

        if (multiThreaded)
        {
            colCheckMutex.unlock();
        }

        return d;
    }

    float CSpace::getDirectedMaxMovement(const Eigen::VectorXf& config, const Eigen::VectorXf& nextConfig)
    {
        SABA_ASSERT(config.rows() == dimension)
        SABA_ASSERT(nextConfig.rows() == dimension)

        SABA_WARNING << "NYI..." << endl;
        // todo : oobb updates in VirtualRobot
        return 0.0f;

        /*if (multiThreaded)
            colCheckMutex.lock();

        robotNodes->setJointValues(config);

        float directedFreeSpaceStep = 0.0f;
        for (unsigned int i=0;i<dimension;i++)
        {
            if (robotJoints[i]->isRotationalJoint())
                directedFreeSpaceStep += robotJoints[i]->getMaxDistanceOOBB() * fabs(nextConfig[i] - config[i]);
            else if (robotJoints[i]->isTranslationalJoint())
                directedFreeSpaceStep += fabs(nextConfig[i] - config[i]);
        }

        if (multiThreaded)
            colCheckMutex.unlock();

        return directedFreeSpaceStep;*/
    }


    bool CSpace::isCollisionFree(const Eigen::VectorXf& config)
    {
        SABA_ASSERT(config.rows() == dimension)

        if (multiThreaded)
        {
            colCheckMutex.lock();
        }

        // set configuration (joint values)
        robo->setJointValues(robotNodes, config);
        bool res = cdm->isInCollision();

        if (multiThreaded)
        {
            colCheckMutex.unlock();
        }

        performaceVars_collisionCheck++;
        return !res;
    }

    void CSpace::lock()
    {
        //if (multiThreaded)
        colCheckMutex.lock();
    }

    void CSpace::unlock()
    {
        //if (multiThreaded)
        colCheckMutex.unlock();
    }

    bool CSpace::isInBoundary(const Eigen::VectorXf& config)
    {
        SABA_ASSERT(config.rows() == dimension)

        // check boundaries
        for (unsigned int i = 0; i < dimension; i++)
        {
            if (config[i] < boundaryMin[i] || config[i] > boundaryMax[i])
            {
                return false;
            }
        }

        return true;
    }

    bool CSpace::isConfigValid(const Eigen::VectorXf& config, bool checkBorders, bool checkCollisions, bool checkConstraints)
    {
        SABA_ASSERT(config.rows() == dimension)

        // check boundaries
        if (checkBorders && !isInBoundary(config))
        {
            return false;
        }

        // check collision
        if (checkCollisions && !isCollisionFree(config))
        {
            return false;
        }

        //check constraints
        if (checkConstraints && !isSatisfyingConstraints(config))
        {
            return false;
        }

        return true;
    }


    float CSpace::getBoundaryMin(unsigned int d)
    {
        if (d >= dimension)
        {
            SABA_ERROR << "Error: getBoundaryMin: could not get boundary dist.." << std::endl;
            return 0.0f;
        }

        return boundaryMin[d];
    }


    float CSpace::getBoundaryMax(unsigned int d)
    {
        if (d >= dimension)
        {
            SABA_ERROR << "Error: getBoundaryMax: could not get boundary dist.." << std::endl;
            return 0.0f;
        }

        return boundaryMax[d];
    }


    float CSpace::getBoundaryDist(unsigned int d)
    {
        if (d >= dimension)
        {
            SABA_ERROR << "Error: getBoundaryDist: could not get boundary dist.." << std::endl;
            return 0.0f;
        }

        return boundaryDist[d];
    }



    void CSpace::respectBoundaries(Eigen::VectorXf& config)
    {
        SABA_ASSERT(config.rows() == dimension)

        for (unsigned int i = 0; i < dimension; i++)
        {
            if (config[i] < boundaryMin[i])
            {
                config[i] = boundaryMin[i];
            }

            if (config[i] > boundaryMax[i])
            {
                config[i] = boundaryMax[i];
            }
        }
    }


    // config values are not set! (except id)
    CSpaceNodePtr CSpace::createNewNode()
    {
        if (freeNodes.size() == 0)
        {
            SABA_ERROR << " Could not create new nodes... (maxNodes exceeded:" << maxNodes << ")" << std::endl;
            return CSpaceNodePtr();
        }

        // get free node
        CSpaceNodePtr result = freeNodes.back();
        freeNodes.pop_back();

        result->children.clear();
        result->allocated = true;

        return result;
    }

    Eigen::VectorXf CSpace::interpolate(const Eigen::VectorXf& q1, const Eigen::VectorXf& q2, float step)
    {
        SABA_ASSERT(q1.rows() == dimension)
        SABA_ASSERT(q2.rows() == dimension)
        Eigen::VectorXf res(dimension);

        for (unsigned int i = 0; i < dimension; i++)
        {
            res[i] = interpolate(q1, q2, i, step);
        }

        return res;
    }

    float CSpace::interpolate(const Eigen::VectorXf& q1, const Eigen::VectorXf& q2, int dim, float step)
    {
        SABA_ASSERT(q1.rows() == dimension)
        SABA_ASSERT(q2.rows() == dimension)
        SABA_ASSERT_MESSAGE((dim >= 0 && dim < (int)robotJoints.size()),  "Dim " << dim << " out of bounds..." << endl)

        // translational joint
        if (robotJoints[dim]->isTranslationalJoint())
        {
            return interpolateLinear(q1[dim], q2[dim], step);
        }

        // rotational joint
        if (!borderLessDimension[dim])
        {
            return interpolateLinear(q1[dim], q2[dim], step);
        }
        else
        {
            // borderless mode

            float start = q1[dim] - robotJoints[dim]->getJointLimitLo();
            float end = q2[dim] - robotJoints[dim]->getJointLimitLo();
            float res = interpolateRotational(start, end , step);
            res = (float)fmod((double)res, 2.0 * M_PI);
            res = res + robotJoints[dim]->getJointLimitLo();
            return res;
        }

    }

    float CSpace::interpolateLinear(float a, float b, float step)
    {
        return (a + step * (b - a));
    }

    float CSpace::interpolateRotational(float a, float b, float step)
    {
        //return (a + step * (b - a));

        float angle;

        if (fabs(a - b) < M_PI)
        {
            //std::cout << "interpolateRotational: If 1" << std::endl;
            angle = interpolateLinear(a, b, step);
        }
        else if (a < b)
        {
            //std::cout << "interpolateRotational: If 2" << std::endl;
            angle = a - step * (a + 2.0f * (float)M_PI - b);

        }
        else
        {
            //std::cout << "interpolateRotational: If 3" << std::endl;
            angle = a + step * (b + 2.0f * (float)M_PI - a);
        }

        if (angle < 0)
        {
            angle += 2.0f * (float)M_PI;
        }

        if (angle >= 2.0f * (float)M_PI)
        {
            angle -= 2.0f * (float)M_PI;
        }

        return angle;
    }


    float CSpace::getRandomConfig_UniformSampling(unsigned int dim)
    {
        SABA_ASSERT(dim <= dimension)

        float res = (float)rand() * randMult; // value from 0 to 1
        res = boundaryMin[dim] + (boundaryDist[dim] * res);
        return res;
    }

    void CSpace::getRandomConfig(Eigen::VectorXf& config, bool checkValid)
    {
        SABA_ASSERT(config.rows() == dimension)

        do
        {
            if (sampleAlgorithm)
            {
                sampleAlgorithm->sample(config, shared_from_this());
            }
            else
            {
                float t;

                for (unsigned int i = 0; i < dimension; i++)
                {
                    t = (float)rand() * randMult; // value from 0 to 1
                    config[i] = boundaryMin[i] + (boundaryDist[i] * t);
                }
            }
        }
        while (checkValid && !isConfigValid(config, false, true, true));
    }


    void CSpace::printConfig(const Eigen::VectorXf& config) const
    {
        if (config.rows() != dimension)
        {
            SABA_ERROR << "Wrong dimensions..." << endl;
        }

        streamsize pr = cout.precision(2);
        cout << "<";

        for (int i = 0; i < config.rows(); i++)
        {
            cout << config[i];

            if (i != config.rows() - 1)
            {
                cout << ",";
            }
        }

        cout << ">" << endl;
        cout.precision(pr);
    }

    void CSpace::exclusiveRobotAccess(bool bGranted)
    {
        multiThreaded = bGranted;
    }

    bool CSpace::hasExclusiveRobotAccess()
    {
        return multiThreaded;
    }

    /*int CSpace::getMaxConfigs() const
    {
        return maxConfigs;
    }*/

    void CSpace::setRandomSampler(SamplerPtr sampler)
    {
        sampleAlgorithm = sampler;
        enableWeights(useMetricWeights); // update sample class
    }

    void CSpace::enableWeights(bool enable)
    {
        useMetricWeights = enable;

        if (sampleAlgorithm)
        {
            if (enable)
            {
                sampleAlgorithm->enableMetricWeights(metricWeights);
            }
            else
            {
                sampleAlgorithm->disableMetricWeights();
            }
        }
    }

    VirtualRobot::RobotNodeSetPtr CSpace::getRobotNodeSet() const
    {
        return robotNodes;
    }

    void CSpace::removeNode(CSpaceNodePtr node)
    {
        SABA_ASSERT(node)

        if (node->ID >= nodes.size())
        {
            SABA_ERROR << " Could not remove node... (corrupt ID:" << node->ID << ")" << std::endl;
            return;
        }

        node->allocated = false;
        freeNodes.push_back(node);
    }

    CSpacePathPtr CSpace::createPath(const Eigen::VectorXf& start, const Eigen::VectorXf& goal)
    {
        SABA_ASSERT(start.rows() == dimension)
        SABA_ASSERT(goal.rows() == dimension)
        CSpacePathPtr p(new CSpacePath(shared_from_this()));
        p->addPoint(start);
        p->addPoint(goal);
        return p;
    }

    Saba::CSpacePathPtr CSpace::createPathUntilInvalid(const Eigen::VectorXf& start, const Eigen::VectorXf& goal, float& storeAddedLength)
    {
        SABA_ASSERT(start.rows() == dimension);
        SABA_ASSERT(goal.rows() == dimension);
        CSpacePathPtr p(new CSpacePath(shared_from_this()));
        storeAddedLength = 0.5f;
        p->addPoint(start);

        if (isConfigValid(goal))
        {
            p->addPoint(goal);
            storeAddedLength = 1.0f;
        }

        return p;
    }

    bool CSpace::isPathValid(const Eigen::VectorXf& q1, const Eigen::VectorXf& q2)
    {
        return isConfigValid(q2);
    }


    bool CSpace::isBorderlessDimension(unsigned int dim) const
    {
        SABA_ASSERT(dim < dimension)

        if (!(robotJoints[dim]->isTranslationalJoint()))
        {
            // rotational joint
            if (abs((double)(robotJoints[dim]->getJointLimitHi() - robotJoints[dim]->getJointLimitLo())) > (1.9999999999999 * M_PI))
            {
                return true;
            }
        }

        return false;
    }

    unsigned int CSpace::getDimension() const
    {
        return dimension;
    }

    VirtualRobot::RobotPtr CSpace::getRobot() const
    {
        return robo;
    }

    void CSpace::addConstraintCheck(ConfigurationConstraintPtr constraint)
    {
        constraints.push_back(constraint);
    }

    bool CSpace::isSatisfyingConstraints(const Eigen::VectorXf& config)
    {
        for (size_t i = 0; i < constraints.size(); i++)
        {
            if (!constraints[i]->isValid(config))
            {
                return false;
            }
        }

        return true;
    }

}
