
#include "CSpacePath.h"
#include <algorithm>
#include <iostream>
#include <fstream>
using namespace std;

namespace Saba
{

    CSpacePath::CSpacePath(CSpacePtr cspace, const std::string& name): Trajectory(cspace ? cspace->getRobotNodeSet() : VirtualRobot::RobotNodeSetPtr(), name)
    {
        this->cspace = cspace;

        if (!cspace)
        {
            THROW_SABA_EXCEPTION("No cpsace...");
        }

        dimension = cspace->getDimension();
    }

    CSpacePath::~CSpacePath()
    {
        reset();
    }




    CSpacePathPtr CSpacePath::clone() const
    {
        CSpacePathPtr res(new CSpacePath(cspace, name));

        for (unsigned int i = 0; i < getNrOfPoints(); i++)
        {
            res->addPoint(getPoint(i));
        }

        return res;
    }


    /*
    bool CSpacePath::movePosition(unsigned int pos, const Eigen::VectorXf &moveVector, int sideSteps)
    {
        if (pos>=getNrOfPoints())
        {
            SABA_ERROR << "CSpacePath::movePosition: wrong pos" << std::endl;
            return false;
        }
        if (moveVector==NULL)
        {
            SABA_ERROR << "CSpacePath::movePosition: NULL moveVec" << std::endl;
            return false;
        }

        Eigen::VectorXf c = getPoint(pos);

        for (unsigned int i=0;i<dimension;i++)
        {
            c[i] += moveVector[i];
        }
        // apply part of moveVector to sideSteps positions next to pos
        if (sideSteps>0 && dimension>0)
        {
            float *tmp = new float[dimension];
            memcpy (tmp,moveVector,sizeof(float)*dimension);
            int posA,posB;
            float *cA,*cB;

            for (int j=1; j<=sideSteps; j++)
            {
                posA = pos - j;
                posB = pos + j;

                // half moveVector
                for (unsigned int i=0;i<dimension;i++)
                {
                    tmp[i] *= 0.5f;
                }

                // left
                if (posA>=0)
                {
                    cA = getPoint(posA);
                    for (unsigned int i=0;i<dimension;i++)
                    {
                        cA[i] += tmp[i];
                    }
                }

                // right
                if (posB<(int)getNrOfPoints())
                {
                    cB = getPoint(posB);
                    for (unsigned int i=0;i<dimension;i++)
                    {
                        cB[i] += tmp[i];
                    }
                }


            } // j
            delete[] tmp;
        } // sidesteps
    */



    float CSpacePath::getLength() const
    {
        return getLength(true);
    }
    float CSpacePath::getLength(bool useCSpaceWeights) const
    {
        return getLength(0, (int)path.size() - 1, useCSpaceWeights);
    }

    // be careful, this method calculates the c-space length of the path, not the workspace length!
    float CSpacePath::getLength(unsigned int startIndex, unsigned int endIndex, bool useCSpaceWeights) const
    {
        if (endIndex < startIndex || endIndex >= path.size())
        {
            SABA_ERROR << "CSpacePath::getLength: wrong index..." << std::endl;
            return 0.0f;
        }

        float pathLength = 0.0f;
        Eigen::VectorXf c1, c2;
        float l;

        for (unsigned int i = startIndex; i < endIndex; i++)
        {
            c1 = path[i];
            c2 = path[i + 1];

            l = cspace->calcDist(c1, c2, useCSpaceWeights);
            pathLength += l;
        }

        return pathLength;
    }
    /*
    int CSpacePath::checkIntermediatePositions(unsigned int startPos, const float *samplingDist)
    {

        unsigned int i = 0;
        if (startPos>= path.size()-1)
        {
            std::cout << "CSpacePath::checkIntermediatePositions: wrong position" << std::endl;
            return 0;
        }
        int nrOfNewPos = 0;

        // compute difference of each dimension
        float *diff = new float[dimension];
        MathHelpers::calcDiff(path[startPos],path[startPos+1],diff,dimension);

        // compute intermediate step for each dimension, search for maximum step in all dimensions
        int *step = new int[dimension];
        int maxStep = -1;
        bool intermediateStepsNecessary = false;
        for (i=0;i<dimension;i++)
        {
            if (fabs(diff[i])>samplingDist[i]) intermediateStepsNecessary = true; // intermediate steps necessary!
            step[i] =(int)(floor(fabs(diff[i] / samplingDist[i])));
            if (maxStep<step[i]) maxStep = step[i];
        }

        float *actPos = new float[dimension];
        float *goalPos = new float[dimension];
        memcpy(actPos,path[startPos],sizeof(float)*dimension); // copy start pos
        memcpy(goalPos,path[startPos+1],sizeof(float)*dimension); // copy goal pos

        if (intermediateStepsNecessary)
        {
            // apply these maxSteps and create new path configurations except for path[startPos+1]
            for (i=1;(int)i<maxStep;i++)
            {
                float factor = (float)i / (float)maxStep;
                for (unsigned int j=0;j<dimension;j++)
                {
                    actPos[j] = path[startPos][j] + (goalPos[j] - path[startPos][j]) * factor;// go on one step
                }
                insertPosition(startPos+i,actPos);
                nrOfNewPos++;
            }
        }

        // finally create a new path configuration to hold path[startPos+1]
        for (unsigned int j=0;j<dimension;j++)
        {
            actPos[j] = goalPos[j]; // go on one step
        }
        insertPosition(startPos+i,actPos);
        nrOfNewPos++;

        delete diff;
        delete step;
        delete actPos;
        delete goalPos;

        return nrOfNewPos;
    }

    // returns number of added nodes
    int CSpacePath::checkIntermediatePositions(unsigned int startPos, float samplingDist, CSpaceSampled *cspace)
    {
        if (cspace==NULL)
            return 0;

        float dist = 0.0f;
        unsigned int i;
        int newConfigs = 0;
        dist = cspace->getDist(path[startPos],path[startPos+1]);//MathHelpers::calcWeightedDist(startNode->configuration,config,m_metricWeights,m_nTransDim,m_nRotDim,dimension);
        if (dist==0.0f)
        {
            // nothing to do
            //std::cout << "append path: zero dist to new config!" << std::endl;
            return 0;
        }

        float *lastConfig = path[startPos];
        float *config = path[startPos+1];
        float *actPos = new float[dimension];

        while (dist>samplingDist)
        {
            float factor = samplingDist / dist;
            // create a new node with config, store it in nodeList and set parentID

            // copy values
            for (i=0; i<dimension;i++)
            {
                actPos[i] = cspace->interpolate(lastConfig,config,i,factor);// lastConfig[i] + (config[i] - lastConfig[i])*factor;
            }

            newConfigs++;
            insertPosition(startPos+newConfigs,actPos);
            lastConfig = actPos;
            dist = cspace->getDist(actPos,config);//MathHelpers::calcWeightedDist(newNode->configuration,config,m_metricWeights,m_nTransDim,m_nRotDim,dimension);
        }
        delete[] actPos;
        return newConfigs;
    }
    */

    float CSpacePath::getTime(unsigned int nr)
    {
        if (getNrOfPoints() == 0)
        {
            SABA_ERROR << " no Path.." << std::endl;
        }

        if (nr < 0 || nr > getNrOfPoints() - 1)
        {
            SABA_ERROR << " path entry " << nr << " doesnt exist" << std::endl;

            if (nr < 0)
            {
                nr = 0;
            }

            if (nr > getNrOfPoints() - 1)
            {
                nr = getNrOfPoints() - 1;
            }
        }

        float t = 0.0f;

        float l = getLength();

        Eigen::VectorXf c1 = getPoint(0);

        for (unsigned int i = 0; i < nr; i++)
        {
            Eigen::VectorXf c2 = getPoint(i + 1);
            t += cspace->calcDist(c1, c2) / l;
            //t += MathHelpers::calcDistRotational(c1, c2, dimension, m_rotationalDimension)/l;
            c1 = c2;
        }

        return t;
    }

    // returns position on path for time t (0<=t<=1)
    void CSpacePath::interpolate(float t, Eigen::VectorXf& storePathPos, int* storeIndex /*= NULL*/) const
    {
        storePathPos.resize(dimension);

        if (t < 0 || t > 1.0f)
        {
            // check for rounding errors
            if (t < -0.000000001 || t > 1.000001f)
            {
                std::cout << "CSpacePath::interpolatePath: need t value between 0 and 1... (" << t << ")" << std::endl;
            }

            if (t < 0)
            {
                t = 0.0f;
            }

            if (t > 1.0f)
            {
                t = 1.0f;
            }
        }

        if (getNrOfPoints() == 0)
        {
            SABA_WARNING << "CSpacePath::interpolatePath: no Path.." << std::endl;
        }

        if (t == 0.0f)
        {
            storePathPos = getPoint(0);

            if (storeIndex != NULL)
            {
                *storeIndex = 0;
            }

            return;
        }
        else if (t == 1.0f)
        {
            storePathPos = getPoint(getNrOfPoints() - 1);

            if (storeIndex != NULL)
            {
                *storeIndex = (int)path.size();
            }

            return;
        }


        float l = getLength();
        float wantedLength = l * t;
        float actLength = 0.0f;
        unsigned int startIndex = 0;
        Eigen::VectorXf c1 = getPoint(startIndex);
        Eigen::VectorXf c2 = c1;
        float lastLength = 0.0f;

        // search path segment for wantedLength
        while (actLength < wantedLength && startIndex < getNrOfPoints() - 1)
        {
            c1 = c2;
            startIndex++;
            c2 = getPoint(startIndex);
            lastLength = cspace->calcDist(c1, c2);
            //lastLength = MathHelpers::calcDistRotational(c1, c2, dimension, m_rotationalDimension);
            actLength += lastLength;
        }

        startIndex--;
        actLength -= lastLength;

        // segment starts with startIndex
        float restLength = wantedLength - actLength;

        float factor = 0.0f;

        if (lastLength > 0)
        {
            factor = restLength / lastLength;
        }

        if (factor > 1.0f)
        {
            // ignore rounding errors
            factor = 1.0f;
        }

        for (unsigned int j = 0; j < dimension; j++)
        {
            if (cspace->isBorderlessDimension(j))
            {
                float diff = (c2[j] - c1[j]);

                if (diff > M_PI)
                {
                    diff -= (float)M_PI * 2.0f;
                }

                if (diff < -M_PI)
                {
                    diff += (float)M_PI * 2.0f;
                }

                storePathPos[j] = c1[j] +  diff * factor; // storePos = startPos + factor*segment
            }
            else
            {
                storePathPos[j] = c1[j] + (c2[j] - c1[j]) * factor; // storePos = startPos + factor*segment
            }
        }

        if (storeIndex != NULL)
        {
            *storeIndex = startIndex;
        }

    }
    /*
    void CSpacePath::print() const
    {
        std::cout << "<CSpacePath size='" << path.size() << "' dim='" << dimension << "'>" << std::endl << std::endl;
        for (unsigned int i = 0; i < path.size(); i++)
        {
            std::cout << "\t<Node id='" << i << "'>" << std::endl;
            Eigen::VectorXf c = path[i];
            for (unsigned int k = 0; k < dimension; k++)
            {
                std::cout << "\t\t<c value='" << c[k] << "'/>" << std::endl;
            }
            std::cout << "\t</Node>" << std::endl << std::endl;
        }

        std::cout << "</CSpacePath>" << std::endl;
    }*/

    Saba::CSpacePtr CSpacePath::getCSpace()
    {
        return cspace;
    }

    std::vector<Eigen::Matrix4f > CSpacePath::createWorkspacePath(VirtualRobot::RobotNodePtr r)
    {
        VR_ASSERT(cspace);

        std::vector<Eigen::Matrix4f > result;

        if (cspace->hasExclusiveRobotAccess())
        {
            CSpace::lock();
        }

        result = VirtualRobot::Trajectory::createWorkspaceTrajectory(r);

        if (cspace->hasExclusiveRobotAccess())
        {
            CSpace::unlock();
        }

        return result;
    }

} // namespace Saba
