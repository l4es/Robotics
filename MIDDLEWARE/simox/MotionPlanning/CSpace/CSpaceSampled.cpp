
#include "CSpaceSampled.h"
#include "CSpaceNode.h"
#include "CSpacePath.h"
#include "CSpaceTree.h"
#include "VirtualRobot/Robot.h"
//#include "MathHelpers.h"
#include <cmath>
#include "float.h"
#include <iostream>
#include <string>

using namespace std;

//#define LOCAL_DEBUG(a) {SABA_INFO << a;};
#define LOCAL_DEBUG(a)

namespace Saba
{

    CSpaceSampled::CSpaceSampled(VirtualRobot::RobotPtr robot, VirtualRobot::CDManagerPtr collisionManager, VirtualRobot::RobotNodeSetPtr robotNodes, unsigned int maxConfigs, unsigned int randomSeed)
        : CSpace(robot, collisionManager, robotNodes, maxConfigs, randomSeed),
          recursionMaxDepth(1000)
    {
        recursiveTmpValuesIndex = 0;
        samplingSizePaths = 0.1f;
        samplingSizeDCD = 0.1f;
        SABA_ASSERT(dimension != 0);

        checkPathConfig.setZero(dimension);
        tmpConfig.setZero(dimension);

        for (int i = 0; i < recursionMaxDepth; i++)
        {
            recursiveTmpValues.push_back(tmpConfig);
        }
    }


    CSpaceSampled::~CSpaceSampled()
    {
    }

    CSpacePtr CSpaceSampled::clone(VirtualRobot::CollisionCheckerPtr newColChecker, VirtualRobot::RobotPtr newRobot, VirtualRobot::CDManagerPtr newCDM, unsigned int newRandomSeed)
    {
        cloneCounter++;
        SABA_ASSERT(newColChecker && newRobot && newCDM && newRobot->getRootNode());

        if (newRobot->getRootNode()->getCollisionChecker() != newColChecker)
        {
            SABA_ERROR << ": Collision Checker instances do not match..." << endl;
            return CSpaceSampledPtr();
        }

        CSpaceSampledPtr result(new CSpaceSampled(newRobot, newCDM, robotNodes, newRandomSeed));
        result->setBoundaries(boundaryMin, boundaryMax);

        if (useMetricWeights)
        {
            result->setMetricWeights(metricWeights);
        }

        // sampled cspace
        result->setSamplingSize(getSamplingSize());
        result->setSamplingSizeDCD(getSamplingSizeDCD());

        // todo
        return (CSpacePtr)result;
    }

    void CSpaceSampled::setSamplingSize(float fSize)
    {
        //std::cout << "Setting sampling step size to " << samplingSize << std::endl;
        samplingSizePaths = fSize;
    }

    void CSpaceSampled::setSamplingSizeDCD(float fSize)
    {
        //std::cout << "Setting col check sampling step size to " << samplingSize << std::endl;
        samplingSizeDCD = fSize;
    }


    CSpacePathPtr CSpaceSampled::createPath(const Eigen::VectorXf& start, const Eigen::VectorXf& goal)
    {
        SABA_ASSERT(start.rows() == dimension);
        SABA_ASSERT(goal.rows() == dimension);
        CSpacePathPtr p(new CSpacePath(shared_from_this()));
        p->addPoint(start);

        float dist = 0.0f;
        dist = calcDist(start, goal);

        if (dist == 0.0f)
        {
            // nothing to do
            return p;
        }

        Eigen::VectorXf lastConfig = start;
        float samplingSize = getSamplingSize();
        bool endLoop = false;
        bool lastNode = false;
        LOCAL_DEBUG("Start create path with sampling size:" << samplingSize)
        float factor;

        if (dist < samplingSize)
        {
            lastNode = true;
        }

        do
        {
            LOCAL_DEBUG("-------------------------------------------------------" << endl);
            LOCAL_DEBUG("DIST:" << dist << endl);

            if (lastNode)
            {
                LOCAL_DEBUG("LAST NODE" << endl);
                lastConfig = goal;
            }
            else
            {
                factor = samplingSize / dist;
                LOCAL_DEBUG("dist:" << dist << ", factor:" << factor << endl)
                lastConfig = interpolate(lastConfig, goal, factor);
                LOCAL_DEBUG("lastConfig:" << lastConfig << endl);
            }

            p->addPoint(lastConfig);
            dist = calcDist(lastConfig, goal);

            if (dist <= samplingSize * 1.05f)
            {
                LOCAL_DEBUG("dist<samplingSize" << endl);

                if (lastNode || dist < 1e-6)
                {
                    LOCAL_DEBUG("END LOOP" << endl)
                    endLoop = true;
                }

                lastNode = true;
            }

        }
        while (!endLoop);

        return p;
    }

    Saba::CSpacePathPtr CSpaceSampled::createPathUntilInvalid(const Eigen::VectorXf& start, const Eigen::VectorXf& goal, float& storeAddedLength)
    {
        SABA_ASSERT(start.rows() == dimension);
        SABA_ASSERT(goal.rows() == dimension);
        CSpacePathPtr p(new CSpacePath(shared_from_this()));
        p->addPoint(start);
        LOCAL_DEBUG("Start:" << endl << start << endl);
        LOCAL_DEBUG("Goal:" << endl << goal << endl);
        storeAddedLength = 0.0f;


        float dist = 0.0f;
        dist = calcDist(start, goal);
        float origDist = dist;

        if (dist == 0.0f)
        {
            // nothing to do
            storeAddedLength = 1.0f;
            return p;
        }

        Eigen::VectorXf lastConfig = start;

        // init tmp values
        tmpConfig = start;

        float nodeDist = 0.0f;
        float colCheckDist = getSamplingSizeDCD();
        float samplingSize = getSamplingSize();
        bool endLoop = false;
        bool lastNode = false;

        do
        {
            if (dist <= colCheckDist)
            {
                LOCAL_DEBUG("END" << endl);

                if (!(isConfigValid(goal, false, true, true)))
                {
                    LOCAL_DEBUG("not col free" << endl);
                    storeAddedLength = (origDist - dist) / origDist;
                    LOCAL_DEBUG("not col free, length: " << storeAddedLength << endl);
                    return p;
                }

                LOCAL_DEBUG("col free" << endl);
                storeAddedLength = 1.0f;
                p->addPoint(goal);
                endLoop = true;
            }
            else
            {
                // check node
                LOCAL_DEBUG("step from " << endl << tmpConfig << endl);
                generateNewConfig(goal, tmpConfig, tmpConfig, colCheckDist, dist);
                LOCAL_DEBUG("--> " << endl << tmpConfig << endl);

                if (!(isConfigValid(tmpConfig, false, true, true)))
                {
                    return p;
                }

                nodeDist += colCheckDist;

                if (nodeDist >= samplingSizePaths)
                {
                    // create a new node with config, store it in nodeList and set parentID
                    p->addPoint(tmpConfig);
                    storeAddedLength = (origDist - dist) / origDist;
                    LOCAL_DEBUG("addPoint (length: " << storeAddedLength << ") " << endl << tmpConfig << endl);
                    nodeDist -= samplingSizePaths;
                    LOCAL_DEBUG("addPoint (nodeDist: " << nodeDist << ")" << endl);
                }

                float distT = calcDist(tmpConfig, goal);
                SABA_ASSERT(distT < dist);
                dist = distT;
            }

        }
        while (!endLoop);

        return p;
    }


    bool CSpaceSampled::isPathValid(const Eigen::VectorXf& q1, const Eigen::VectorXf& q2)
    {
        if (stopPathCheck)
        {
            return false;
        }

        // actual weighted distance for collision checking
        float actualWeightedDistance = calcDist(q1, q2);

        if (actualWeightedDistance <= samplingSizeDCD * 1.001f)
        {
            // just checking q2, to avoid double checks
            return isConfigValid(q2, false, true, true); // assuming that q1 and q2 are within the limits of the CSpace
        }
        else
        {
            recursiveTmpValuesIndex++;

            if (recursiveTmpValuesIndex >= recursionMaxDepth)
            {
                SABA_WARNING << "Recursion depth too high: " << recursiveTmpValuesIndex << std::endl;
                recursiveTmpValuesIndex--;
                return false;
            }

            Eigen::VectorXf& middleConfiguration = recursiveTmpValues[recursiveTmpValuesIndex];

            // generate a configuration exactly in the middle of q1 and q2 using weighted distances
            generateNewConfig(q2, q1, middleConfiguration, actualWeightedDistance * 0.5f, actualWeightedDistance);

            bool r = (isPathValid(q1, middleConfiguration) && isPathValid(middleConfiguration, q2));
            recursiveTmpValuesIndex--;
            return r;
        }
    }

} // Saba
