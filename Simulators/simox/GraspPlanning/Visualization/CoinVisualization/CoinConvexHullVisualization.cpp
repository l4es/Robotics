
#include "CoinConvexHullVisualization.h"
#include "../../ConvexHullGenerator.h"

#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoNormal.h>


#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoFaceSet.h>

#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>


namespace GraspStudio
{

    CoinConvexHullVisualization::CoinConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr convHull, bool useFirst3Coords) :
        ConvexHullVisualization(convHull, useFirst3Coords)
    {
        buildVisu();
    }

    CoinConvexHullVisualization::CoinConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr convHull) :
        ConvexHullVisualization(convHull)
    {
        buildVisu();
    }

    void CoinConvexHullVisualization::buildVisu()
    {
        visualization = new SoSeparator();
        visualization->ref();

        if (convHull3D)
        {
            SoSeparator* vi = createConvexHullVisualization(convHull3D);

            if (vi)
            {
                visualization->addChild(vi);
            }

        }

        if (convHull6D)
        {
            SoSeparator* vi = createConvexHullVisualization(convHull6D, useFirst3Coords);

            if (vi)
            {
                visualization->addChild(vi);
            }
        }
    }

    /**
     * If CoinConvexHullVisualization::visualization is a valid object call SoNode::unref()
     * on it.
     */
    CoinConvexHullVisualization::~CoinConvexHullVisualization()
    {
        if (visualization)
        {
            visualization->unref();
        }

    }



    /**
     * This mehtod returns the internal CoinConvexHullVisualization::visualization.
     */
    SoSeparator* CoinConvexHullVisualization::getCoinVisualization()
    {
        return visualization;
    }


    SoSeparator* CoinConvexHullVisualization::createConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr& convHull)
    {
        SoSeparator* result = new SoSeparator;

        if (!convHull || convHull->vertices.size() == 0 || convHull->faces.size() == 0)
        {
            return result;
        }

        result->ref();

        SoUnits* u = new SoUnits;
        u->units = SoUnits::MILLIMETERS;
        result->addChild(u);

        SoCoordinate3* pCoords = new SoCoordinate3();
        SoFaceSet* pFaceSet = new SoFaceSet();

        int nFaces = (int)convHull->faces.size();
        int nVertices = nFaces * 3;
        //Face3D f;
        //Vec3D v1,v2,v3;

         //SoNormal *pNormals = new SoNormal;
         //SbVec3f* normalsArray = new SbVec3f[nFaces];

        // compute points and normals
        SbVec3f* pVertexArray = new SbVec3f[nVertices];
        int nVertexCount = 0;

        for (int i = 0; i < nFaces; i++)
        {
            Eigen::Vector3f &v1 = convHull->vertices.at(convHull->faces[i].id1);
            Eigen::Vector3f &v2 = convHull->vertices.at(convHull->faces[i].id2);
            Eigen::Vector3f &v3 = convHull->vertices.at(convHull->faces[i].id3);

            Eigen::Vector3f &n = convHull->faces[i].normal;

            bool bNeedFlip = ConvexHullGenerator::checkVerticeOrientation(v1, v2, v3, n);

            // COUNTER CLOCKWISE
            if (bNeedFlip)
            {
                pVertexArray[nVertexCount].setValue((float)v3(0), (float)v3(1), (float)v3(2));
            }
            else
            {
                pVertexArray[nVertexCount].setValue((float)v1(0), (float)v1(1), (float)v1(2));
            }

            nVertexCount++;
            pVertexArray[nVertexCount].setValue((float)v2(0), (float)v2(1), (float)v2(2));
            nVertexCount++;

            if (bNeedFlip)
            {
                pVertexArray[nVertexCount].setValue((float)v1(0), (float)v1(1), (float)v1(2));
            }
            else
            {
                pVertexArray[nVertexCount].setValue((float)v3(0), (float)v3(1), (float)v3(2));
            }

            nVertexCount++;

            /*if (bNeedFlip)
            {
                normalsArray[i][0] = n(0);
                normalsArray[i][1] = n(1);
                normalsArray[i][2] = n(2);
            } else
            {
                normalsArray[i][0] = -n(0);
                normalsArray[i][1] = -n(1);
                normalsArray[i][2] = -n(2);
            }*/
            //VR_INFO << "Face " << i << ": v1: " << v1(0) << "," << v1(1) << "," << v1(2) << endl;
            //VR_INFO << "     " << i << ": v2: " << v2(0) << "," << v2(1) << "," << v2(2) << endl;
            //VR_INFO << "     " << i << ": v3: " << v3(0) << "," << v3(1) << "," << v3(2) << endl;


        }

        // set normals
        //pNormals->vector.setValues(0, nFaces, normalsArray);
        //result->addChild(pNormals);
        SoNormalBinding *pNormalBinding = new SoNormalBinding;
        pNormalBinding->value = SoNormalBinding::NONE;
        result->addChild(pNormalBinding);

        // set points
        pCoords->point.setValues(0, nVertices, pVertexArray);
        result->addChild(pCoords);

        // set faces
        int32_t* nNumVertices = new int32_t[nFaces];

        for (int i = 0; i < nFaces; i++)
        {
            nNumVertices[i] = 3;
        }
        pFaceSet->numVertices.setValues(0, nFaces, nNumVertices);

        pFaceSet->numVertices.setValues(0, nFaces, (const int32_t*)nNumVertices);

        result->addChild(pCoords);
        result->addChild(pFaceSet);
        //delete []pVertexArray;
        //delete []nNumVertices;
        //result->unrefNoDelete();

        return result;
    }

    /*
    void addVertex(Vec3D &v1,Vec3D &v2,Vec3D &v3,Vec3D &normal,SbVec3f *pVertexArray, int& nVertexCount)
    {
        bool bNeedFlip = GraspStudioHelpers::checkVerticeOrientation(v1,v2,v3,normal);

        // COUNTER CLOCKWISE
        if (bNeedFlip)
            pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
        else
            pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
        nVertexCount++;

        pVertexArray[nVertexCount].setValue((float)v2.x,(float)v2.y,(float)v2.z);
        nVertexCount++;

        if (bNeedFlip)
            pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
        else
            pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
        nVertexCount++;
    }*/


    SoSeparator* CoinConvexHullVisualization::createConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr& convHull, bool buseFirst3Coords)
    {
        SoSeparator* result = new SoSeparator;

        if (!convHull || convHull->vertices.size() == 0 || convHull->faces.size() == 0)
        {
            return result;
        }

        result->ref();

        SoUnits* u = new SoUnits;
        u->units = SoUnits::MILLIMETERS;
        result->addChild(u);

        SoScale* sc = new SoScale();
        float fc = 400.0f;
        sc->scaleFactor.setValue(fc, fc, fc);
        result->addChild(sc);


        int nFaces = (int)convHull->faces.size();


        //Eigen::Vector3f v[6];
        // project points to 3d, then create hull of these points to visualize it
        std::vector<Eigen::Vector3f> vProjectedPoints;

        for (int i = 0; i < nFaces; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (buseFirst3Coords)
                {
                    //v[j] = convHull->vertices.at(convHull->faces[i].id[j]).p;
                    vProjectedPoints.push_back(convHull->vertices[ convHull->faces[i].id[j] ].p);
                }
                else
                {
                    //v[j] = convHull->vertices.at(convHull->faces[i].id[j]).n;
                    vProjectedPoints.push_back(convHull->vertices[ convHull->faces[i].id[j] ].n);
                }

            }
        }

        VirtualRobot::MathTools::ConvexHull3DPtr projectedHull = ConvexHullGenerator::CreateConvexHull(vProjectedPoints);

        if (!projectedHull)
        {
            GRASPSTUDIO_ERROR << " Could not create hull of projected points, aborting..." << endl;
            return result;
        }

        SoSeparator* hullV = createConvexHullVisualization(projectedHull);

        if (!hullV)
        {
            GRASPSTUDIO_ERROR << " Could not create visualization of projected points, aborting..." << endl;
            return result;
        }

        result->addChild(hullV);
        result->unrefNoDelete();
        return result;
        /*
            // creates 3d-projection of all 6d facets
            int nVertices = nFaces*12;
        SoCoordinate3* pCoords = new SoCoordinate3();
        SoFaceSet* pFaceSet = new SoFaceSet();
        Face6d f;
        Vec3d v1,v2,v3,v4,v5,v6;
        Vec3d normal;

        SbVec3f *pVertexArray = new SbVec3f[nVertices];

        int nVertexCount = 0;
        bool bNeedFlip = false;

        for (int i=0;i<nFaces;i++)
        {
            f = convHull.faces.at(i);
            if (buseFirst3Coords)
            {
                v1.x = convHull.vertices.at(f.id[0]).x;
                v1.y = convHull.vertices.at(f.id[0]).y;
                v1.z = convHull.vertices.at(f.id[0]).z;
                v2.x = convHull.vertices.at(f.id[1]).x;
                v2.y = convHull.vertices.at(f.id[1]).y;
                v2.z = convHull.vertices.at(f.id[1]).z;
                v3.x = convHull.vertices.at(f.id[2]).x;
                v3.y = convHull.vertices.at(f.id[2]).y;
                v3.z = convHull.vertices.at(f.id[2]).z;
                v4.x = convHull.vertices.at(f.id[3]).x;
                v4.y = convHull.vertices.at(f.id[3]).y;
                v4.z = convHull.vertices.at(f.id[3]).z;
                v5.x = convHull.vertices.at(f.id[4]).x;
                v5.y = convHull.vertices.at(f.id[4]).y;
                v5.z = convHull.vertices.at(f.id[4]).z;
                v6.x = convHull.vertices.at(f.id[5]).x;
                v6.y = convHull.vertices.at(f.id[5]).y;
                v6.z = convHull.vertices.at(f.id[5]).z;
                normal.x = f.normal.x;
                normal.y = f.normal.y;
                normal.z = f.normal.z;
            } else
            {
                v1.x = convHull.vertices.at(f.id[0]).nx;
                v1.y = convHull.vertices.at(f.id[0]).ny;
                v1.z = convHull.vertices.at(f.id[0]).nz;
                v2.x = convHull.vertices.at(f.id[1]).nx;
                v2.y = convHull.vertices.at(f.id[1]).ny;
                v2.z = convHull.vertices.at(f.id[1]).nz;
                v3.x = convHull.vertices.at(f.id[2]).nx;
                v3.y = convHull.vertices.at(f.id[2]).ny;
                v3.z = convHull.vertices.at(f.id[2]).nz;
                v4.x = convHull.vertices.at(f.id[3]).nx;
                v4.y = convHull.vertices.at(f.id[3]).ny;
                v4.z = convHull.vertices.at(f.id[3]).nz;
                v5.x = convHull.vertices.at(f.id[4]).nx;
                v5.y = convHull.vertices.at(f.id[4]).ny;
                v5.z = convHull.vertices.at(f.id[4]).nz;
                v6.x = convHull.vertices.at(f.id[5]).nx;
                v6.y = convHull.vertices.at(f.id[5]).ny;
                v6.z = convHull.vertices.at(f.id[5]).nz;
                normal.x = f.normal.nx;
                normal.y = f.normal.ny;
                normal.z = f.normal.nz;
            }
            bool bNeedFlip = GraspStudioHelpers::checkVerticeOrientation(v1,v2,v3,normal);
            if (bNeedFlip)
            {
                pVertexArray[nVertexCount].setValue((float)v6.x,(float)v6.y,(float)v6.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v5.x,(float)v5.y,(float)v5.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v4.x,(float)v4.y,(float)v4.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v2.x,(float)v2.y,(float)v2.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
                nVertexCount++;
            } else
            {
                pVertexArray[nVertexCount].setValue((float)v1.x,(float)v1.y,(float)v1.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v2.x,(float)v2.y,(float)v2.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v3.x,(float)v3.y,(float)v3.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v4.x,(float)v4.y,(float)v4.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v5.x,(float)v5.y,(float)v5.z);
                nVertexCount++;
                pVertexArray[nVertexCount].setValue((float)v6.x,(float)v6.y,(float)v6.z);
                nVertexCount++;
            }
        }
        pCoords->point.setValues(0,nVertices,pVertexArray);
        long *nNumVertices = new long[nFaces];
        for (int i=0;i<nFaces;i++)
            nNumVertices[i] = 6;
        pFaceSet->numVertices.setValues(0,nFaces,(const int32_t*)nNumVertices);

        pStoreResult->addChild(pCoords);
        pStoreResult->addChild(pFaceSet);
        delete []pVertexArray;
        delete []nNumVertices;

        qhull_mutex.unlock();

        return true;
        */
    }


} // namespace Saba
