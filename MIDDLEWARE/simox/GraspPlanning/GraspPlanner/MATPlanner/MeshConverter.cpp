/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2013 H2T,KIT
*             GNU Lesser General Public License
*
*/
#include "MeshConverter.h"
#include <VirtualRobot/Visualization/VisualizationFactory.h>

using namespace std;
using namespace Eigen;
using namespace VirtualRobot;

namespace GraspStudio
{


    int MeshConverter::hasVertex(std::vector< Eigen::Vector3f>& vectList, Eigen::Vector3f& obj)
    {
        for (size_t j = 0; j < vectList.size(); j++)
        {
            if (vectList[j] == obj)
            {
                return j;
            }
        }

        return -1;
    }

    VirtualRobot::ObstaclePtr MeshConverter::refineObjectSurface(VirtualRobot::ObstaclePtr object, float maxDist)
    {
        VirtualRobot::ObstaclePtr res;

        if (!object || !object->getCollisionModel())
        {
            return res;
        }

        TriMeshModelPtr tm = object->getCollisionModel()->getTriMeshModel();

        if (!tm)
        {
            return res;
        }

        VR_INFO << "Processing object with " << tm->faces.size() << " triangles" << endl;

        // first create new object
        TriMeshModelPtr triMesh2(new TriMeshModel());
        int check;

        for (size_t i = 0; i < tm->faces.size(); i++)
        {
            Eigen::Vector3f v1, v2, v3;
            v1 = tm->vertices[tm->faces[i].id1];
            v2 = tm->vertices[tm->faces[i].id2];
            v3 = tm->vertices[tm->faces[i].id3];
            unsigned int id1, id2, id3;
            check = hasVertex(triMesh2->vertices, v1);

            if (check > 0)
            {
                id1 = (int)check;
            }
            else
            {
                // add vertex
                triMesh2->addVertex(v1);
                id1 = triMesh2->vertices.size() - 1;
            }

            check = hasVertex(triMesh2->vertices, v2);

            if (check > 0)
            {
                id2 = (int)check;
            }
            else
            {
                // add vertex
                triMesh2->addVertex(v2);
                id2 = triMesh2->vertices.size() - 1;
            }

            check = hasVertex(triMesh2->vertices, v3);

            if (check > 0)
            {
                id3 = (int)check;
            }
            else
            {
                // add vertex
                triMesh2->addVertex(v3);
                id3 = triMesh2->vertices.size() - 1;
            }

            // create face
            MathTools::TriangleFace face;
            face.id1 = id1;
            face.id2 = id2;
            face.id3 = id3;

            face.normal = tm->faces[i].normal;

            triMesh2->addFace(face);
        }


        for (size_t i = 0; i < triMesh2->faces.size(); i++)
        {
            checkAndSplitVertex(triMesh2, i, maxDist);
        }

        VisualizationFactoryPtr cv = VisualizationFactory::first(NULL);

        if (!cv)
        {
            return res;
        }

        Eigen::Matrix4f gp = object->getGlobalPose();
        VisualizationNodePtr visu = cv->createTriMeshModelVisualization(triMesh2, false, gp);
        CollisionModelPtr cm(new CollisionModel(visu));
        res.reset(new Obstacle(object->getName(), visu, cm));
        return res;
    }

    void MeshConverter::checkAndSplitVertex(VirtualRobot::TriMeshModelPtr tm, int faceIdx, float maxDist)
    {
        VR_ASSERT(tm);
        VR_ASSERT(faceIdx >= 0 && faceIdx < (int)tm->faces.size());

        float d12, d13, d23;
        Eigen::Vector3f v1, v2, v3;
        v1 = tm->vertices[tm->faces[faceIdx].id1];
        v2 = tm->vertices[tm->faces[faceIdx].id2];
        v3 = tm->vertices[tm->faces[faceIdx].id3];
        Eigen::Vector3f v4;
        unsigned int id4;
        size_t checkFaceIdx;

        d12 = (v1 - v2).norm();
        d23 = (v2 - v3).norm();
        d13 = (v1 - v3).norm();

        if (d12 > maxDist || d23 > maxDist || d13 > maxDist)
        {
            // split at longest edge
            if (d12 >= d23 && d12 >= d13)
            {
                v4 = v1 + (v2 - v1) * 0.5f;
                tm->addVertex(v4);
                id4 = tm->vertices.size() - 1;

                // add new face
                MathTools::TriangleFace face;
                face.id1 = id4;
                face.id2 = tm->faces[faceIdx].id2;
                face.id3 = tm->faces[faceIdx].id3;
                face.normal = tm->faces[faceIdx].normal;
                tm->addFace(face);

                // update current face
                tm->faces[faceIdx].id2 = id4;
            }
            else if (d23 >= d12 && d23 >= d13)
            {
                v4 = v2 + (v3 - v2) * 0.5f;
                tm->addVertex(v4);
                id4 = tm->vertices.size() - 1;

                // add new face
                MathTools::TriangleFace face;
                face.id1 = tm->faces[faceIdx].id1;
                face.id2 = id4;
                face.id3 = tm->faces[faceIdx].id3;
                face.normal = tm->faces[faceIdx].normal;
                tm->addFace(face);

                // update current face
                tm->faces[faceIdx].id3 = id4;
            }
            else
            {
                v4 = v3 + (v1 - v3) * 0.5f;
                tm->addVertex(v4);
                id4 = tm->vertices.size() - 1;

                // add new face
                MathTools::TriangleFace face;
                face.id1 = tm->faces[faceIdx].id1;
                face.id2 = tm->faces[faceIdx].id2;
                face.id3 = id4;
                face.normal = tm->faces[faceIdx].normal;
                tm->addFace(face);

                // update current face
                tm->faces[faceIdx].id1 = id4;

            }

            checkFaceIdx = tm->faces.size() - 1;
            // check new face
            checkAndSplitVertex(tm, checkFaceIdx, maxDist);
            // recursively check current face
            checkAndSplitVertex(tm, faceIdx, maxDist);
        }
    }

    float MeshConverter::getMaxVertexDistance(VirtualRobot::TriMeshModelPtr tm)
    {
        if (!tm)
        {
            return 0.0f;
        }

        float maxDist = 0;

        for (size_t i = 0; i < tm->faces.size(); i++)
        {
            Eigen::Vector3f v1, v2, v3;
            v1 = tm->vertices[tm->faces[i].id1];
            v2 = tm->vertices[tm->faces[i].id2];
            v3 = tm->vertices[tm->faces[i].id3];

            float d12 = (v1 - v2).norm();
            float d23 = (v2 - v3).norm();
            float d13 = (v1 - v3).norm();

            if (d12 > maxDist)
            {
                maxDist = d12;
            }

            if (d23 > maxDist)
            {
                maxDist = d23;
            }

            if (d13 > maxDist)
            {
                maxDist = d13;
            }
        }

        return maxDist;

    }

}
