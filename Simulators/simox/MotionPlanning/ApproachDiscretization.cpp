#include "ApproachDiscretization.h"
#include <iostream>
#include <time.h>
#include <algorithm>
#include <limits.h>

using namespace std;

namespace Saba
{


    ApproachDiscretization::ApproachDiscretization(float radius, int steps)
    {
        globalPose.setIdentity();

        //clock_t tStart = clock();
        sphereGenerator.reset(new VirtualRobot::SphereApproximator());
        sphereGenerator->generateGraph(sphere, VirtualRobot::SphereApproximator::eOctahedron, steps, radius);
        //buildIVModel();
        //clock_t tEnd = clock();
        //long diffClock = (long)(((float)(tEnd - tStart) / (float)CLOCKS_PER_SEC) * 1000.0);
        //cout << __FUNCTION__ << " Created Sphere in " << diffClock << "ms with " << sphere.m_Vertices.size() << " vertices and " << sphere.faces.size() << " faces "<< endl;
    }

    ApproachDiscretization::~ApproachDiscretization()
    {
    }

    /*
    void ApproachDiscretization::buildIVModel()
    {
        if (m_pIVModel)
            m_pIVModel->unref();
        m_pIVModel = new SoSeparator();
        m_pIVModel->ref();

        SoMatrixTransform *pTrans = new SoMatrixTransform();
        pTrans->matrix.setValue(globalPose);
        m_pIVModel->addChild(pTrans);
        SoSeparator* pSphereIV = m_pSphereGenerator->generateIVModel(sphere,0.0f);
        m_pIVModel->addChild(pSphereIV);
    }
    */

    Eigen::Matrix4f ApproachDiscretization::getGlobalPose() const
    {
        return globalPose;
    }

    void ApproachDiscretization::setGlobalPose(const Eigen::Matrix4f& pose)
    {
        globalPose.setIdentity();
        globalPose.block(0, 3, 3, 1) = pose.block(0, 3, 3, 1);
    }

    int ApproachDiscretization::getNearestVertexId(const Eigen::Matrix4f& pose)
    {
        return getNearestVertexIdVec(pose.block(0, 3, 3, 1));
    }

    int ApproachDiscretization::getNearestVertexIdVec(const Eigen::Vector3f& pos)
    {
        Eigen::Vector3f dir = pos - globalPose.block(0, 3, 3, 1);

        if (dir.norm() < 1e-8)
        {
            return 0;
        }

        int nIndex = sphereGenerator->findVertex(dir, 10000.0f, sphere.vertices);

        return nIndex;
    }

    int ApproachDiscretization::getNearestFaceId(const Eigen::Matrix4f& pose)
    {
        return getNearestFaceIdVec(pose.block(0, 3, 3, 1));
    }

    int ApproachDiscretization::getNearestFaceIdVec(const Eigen::Vector3f& pos)
    {
        Eigen::Vector3f dir = pos - globalPose.block(0, 3, 3, 1);

        if (dir.norm() < 1e-8)
        {
            return 0;
        }

        int nIndex = sphereGenerator->findVertex(dir, 10000.0f, sphere.vertices);

        Eigen::Vector3f zeroPt, storeIntPoint;
        zeroPt.setZero();
        VirtualRobot::SphereApproximator::FaceIndex faceIndx = sphere.mapVerticeIndxToFaceIndx[nIndex];

        for (int i = 0; i < (int)faceIndx.faceIds.size(); i++)
        {
            VirtualRobot::MathTools::TriangleFace f = sphere.faces[faceIndx.faceIds[i]];

            //if  (sphereGenerator->check_intersect_tri(sphere.vertices[f.n1],sphere.m_Vertices[f.m_n2],sphere.m_Vertices[f.m_n3],zeroPt,Pose1,storeIntPoint))
            if (sphereGenerator->check_intersect_tri(sphere.vertices[f.id1], sphere.vertices[f.id2], sphere.vertices[f.id3], zeroPt, dir, storeIntPoint))
            {
                //MarkFace(FaceIndx.m_faceIds[i],true);
                return faceIndx.faceIds[i];
            }
        }

        cout << __FUNCTION__ << " No face found ?!" << endl;
        return -1;
    }

    void ApproachDiscretization::removeCSpaceNode(int faceId, CSpaceNodePtr node)
    {
        if (faceId < 0 || faceId > (int)sphere.faces.size())
        {
            cout << __FUNCTION__ << "wrong face ID :" << faceId << endl;
            return;
        }

        if (faceIdToCSpaceNodesMapping.find(faceId) != faceIdToCSpaceNodesMapping.end())
        {
            std::vector<CSpaceNodePtr>::iterator iter = find(faceIdToCSpaceNodesMapping[faceId].cspaceNodes.begin(), faceIdToCSpaceNodesMapping[faceId].cspaceNodes.end(), node);

            if (iter == faceIdToCSpaceNodesMapping[faceId].cspaceNodes.end())
            {
                cout << __FUNCTION__ << " Warning: Node " << node->ID << " is not mapped to face with id " << faceId << endl;
            }
            else
            {
                faceIdToCSpaceNodesMapping[faceId].cspaceNodes.erase(iter);
            }
        }
        else
        {
            cout << __FUNCTION__ << " Warning: Node " << node->ID << " is not mapped to face with id " << faceId << endl;
        }
    }

    void ApproachDiscretization::removeCSpaceNode(const Eigen::Vector3f& cartPos, CSpaceNodePtr node)
    {
        if (!node)
        {
            cout << __FUNCTION__ << "NULL data..." << endl;
            return;
        }

        int faceId = getNearestFaceIdVec(cartPos);
        removeCSpaceNode(faceId, node);
    }

    void ApproachDiscretization::addCSpaceNode(int faceId, CSpaceNodePtr node)
    {
        if (faceId < 0 || faceId > (int)sphere.faces.size())
        {
            cout << __FUNCTION__ << "wrong face ID :" << faceId << endl;
            return;
        }

        if (faceIdToCSpaceNodesMapping.find(faceId) != faceIdToCSpaceNodesMapping.end())
        {
            std::vector<CSpaceNodePtr>::iterator iter = find(faceIdToCSpaceNodesMapping[faceId].cspaceNodes.begin(), faceIdToCSpaceNodesMapping[faceId].cspaceNodes.end(), node);

            if (iter != faceIdToCSpaceNodesMapping[faceId].cspaceNodes.end())
            {
                cout << __FUNCTION__ << " Warning: Node " << node->ID << " is already mapped to face with id " << faceId << endl;
            }
            else
            {
                faceIdToCSpaceNodesMapping[faceId].cspaceNodes.push_back(node);
            }
        }
        else
        {
            faceIdToCSpaceNodesMapping[faceId].count = 0;
            faceIdToCSpaceNodesMapping[faceId].cspaceNodes.push_back(node);
            activeFaces.push_back(faceId);
        }
    }

    void ApproachDiscretization::addCSpaceNode(const Eigen::Vector3f& pos, CSpaceNodePtr node)
    {
        if (!node)
        {
            cout << __FUNCTION__ << "NULL data..." << endl;
            return;
        }

        int faceId = getNearestFaceIdVec(pos);
        addCSpaceNode(faceId, node);
    }

    void ApproachDiscretization::clearCSpaceNodeMapping()
    {
        faceIdToCSpaceNodesMapping.clear();
        activeFaces.clear();
    }

    CSpaceNodePtr ApproachDiscretization::getGoodRatedNode(int loops)
    {
        int nSize = (int)activeFaces.size();

        if (nSize == 0)
        {
            return CSpaceNodePtr();
        }

        int nBestRanking = INT_MAX;
        int nBestFace = -1;
        int nRandFaceId = 0;
        int nLoopCount = 0;
        std::map<int, CSpaceNodeMapping>::iterator iter;

        while (nLoopCount < loops)
        {
            nRandFaceId = activeFaces[(rand() % nSize)];

            if (faceIdToCSpaceNodesMapping[nRandFaceId].count < nBestRanking && faceIdToCSpaceNodesMapping[nRandFaceId].cspaceNodes.size() > 0)
            {
                nBestRanking = faceIdToCSpaceNodesMapping[nRandFaceId].count;
                nBestFace = nRandFaceId;

                if (nBestRanking == 0)
                {
                    break;
                }
            }

            nLoopCount++;
        }

        if (nBestFace < 0)
        {
            return CSpaceNodePtr();
        }

        // increase ranking
        faceIdToCSpaceNodesMapping[nBestFace].count++;

        // get random cspace node
        int nNodes = (int)faceIdToCSpaceNodesMapping[nBestFace].cspaceNodes.size();

        if (nNodes <= 0)
        {
            return CSpaceNodePtr();
        }

        int nRandNode = rand() % nNodes;
        CSpaceNodePtr node = faceIdToCSpaceNodesMapping[nBestFace].cspaceNodes[nRandNode];

        // remove node from list
        removeCSpaceNode(nBestFace, node);

        return node;
    }

}
