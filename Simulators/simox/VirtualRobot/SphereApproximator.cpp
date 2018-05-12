// *****************************************************************
// Filename:    SphereApproximator.cpp
// Copyright:   Kai Welke, Chair Prof. Dillmann (IAIM),
//              Institute for Computer Science and Engineering (CSE),
//              University of Karlsruhe. All rights reserved.
// Author:      Kai Welke
// Date:        08.07.2008
// *****************************************************************

#include "SphereApproximator.h"
#include <math.h>
#include <iostream>
#include "VirtualRobot/Visualization/TriMeshModel.h"

using namespace std;

namespace VirtualRobot
{

    void SphereApproximator::generateGraph(SphereApproximation& storeResult, EPolyhedronType baseType, int levels, float radius)
    {
        GraphData gd;
        initBasePolyhedron(baseType, gd);
        projectToSphere(radius, gd);

        for (int L = 0 ; L < levels ; L++)
        {
            subDivide(gd);
            projectToSphere(radius, gd);
        }

        // copy contents
        storeResult.vertices = gd.vertices;
        storeResult.faces = gd.faces;
        buildVertexFaceMapping(storeResult);
    }

    void SphereApproximator::buildVertexFaceMapping(SphereApproximation& storeResult)
    {
        for (int i = 0; i < (int)storeResult.faces.size(); i++)
        {
            MathTools::TriangleFace f = storeResult.faces[i];
            FaceIndex fi;

            if (storeResult.mapVerticeIndxToFaceIndx.find(f.id1) == storeResult.mapVerticeIndxToFaceIndx.end())
            {
                fi.faceIds.clear();
            }
            else
            {
                fi = storeResult.mapVerticeIndxToFaceIndx[f.id1];
            }

            fi.faceIds.push_back(i);
            storeResult.mapVerticeIndxToFaceIndx[f.id1] = fi;

            if (storeResult.mapVerticeIndxToFaceIndx.find(f.id2) == storeResult.mapVerticeIndxToFaceIndx.end())
            {
                fi.faceIds.clear();
            }
            else
            {
                fi = storeResult.mapVerticeIndxToFaceIndx[f.id2];
            }

            fi.faceIds.push_back(i);
            storeResult.mapVerticeIndxToFaceIndx[f.id2] = fi;

            if (storeResult.mapVerticeIndxToFaceIndx.find(f.id3) == storeResult.mapVerticeIndxToFaceIndx.end())
            {
                fi.faceIds.clear();
            }
            else
            {
                fi = storeResult.mapVerticeIndxToFaceIndx[f.id3];
            }

            fi.faceIds.push_back(i);
            storeResult.mapVerticeIndxToFaceIndx[f.id3] = fi;
        }
    }

    void SphereApproximator::initBasePolyhedron(EPolyhedronType nBaseType, GraphData& gd)
    {
        gd.vertices.clear();
        gd.faces.clear();

        switch (nBaseType)
        {
            case eTetrahedron:
                generateTetrahedron(gd);
                break;

            case eOctahedron:
                generateOctahedron(gd);
                break;

            case eIcosahedron:
                generateIcosahedron(gd);
                break;

            default:
                VR_ERROR << "Wrong base type\n";
                break;
        }
    }
    void SphereApproximator::setVec(Eigen::Vector3f& v, float x, float y, float z)
    {
        v(0) = x;
        v(1) = y;
        v(2) = z;
    }

    void SphereApproximator::generateTetrahedron(GraphData& gd)
    {
        // vertices of a tetrahedron
        float fSqrt3 = sqrtf(3.0);
        Eigen::Vector3f v[4];

        setVec(v[0], fSqrt3, fSqrt3, fSqrt3);
        setVec(v[1], -fSqrt3, -fSqrt3, fSqrt3);
        setVec(v[2], -fSqrt3, fSqrt3, -fSqrt3);
        setVec(v[3], fSqrt3, -fSqrt3, -fSqrt3);

        for (int i = 0 ; i < 4 ; i++)
        {
            gd.vertices.push_back(v[i]);
        }

        // structure describing a tetrahedron
        MathTools::TriangleFace f[4];
        f[0].set(1, 2, 3);
        f[1].set(1, 4, 2);
        f[2].set(3, 2, 4);
        f[3].set(4, 1, 3);

        for (int i = 0 ; i < 4 ; i++)
        {
            f[i].id1--;
            f[i].id2--;
            f[i].id3--;
            gd.faces.push_back(f[i]);
        }
    }

    void SphereApproximator::generateOctahedron(GraphData& gd)
    {
        // Six equidistant points lying on the unit sphere
        Eigen::Vector3f v[6];
        setVec(v[0], 1, 0, 0);
        setVec(v[1], -1, 0, 0);
        setVec(v[2], 0, 1, 0);
        setVec(v[3], 0, -1, 0);
        setVec(v[4], 0, 0, 1);
        setVec(v[5], 0, 0, -1);

        for (int i = 0 ; i < 6 ; i++)
        {
            gd.vertices.push_back(v[i]);
        }

        // Join vertices to create a unit octahedron
        MathTools::TriangleFace f[8];
        f[0].set(1, 5, 3);
        f[1].set(3, 5, 2);
        f[2].set(2, 5, 4);
        f[3].set(4, 5, 1);
        f[4].set(1, 3, 6);
        f[5].set(3, 2, 6);
        f[6].set(2, 4, 6);
        f[7].set(4, 1, 6);

        for (int i = 0 ; i < 8 ; i++)
        {
            f[i].id1--;
            f[i].id2--;
            f[i].id3--;
            gd.faces.push_back(f[i]);
        }
    }

    void SphereApproximator::generateIcosahedron(GraphData& gd)
    {
        // Twelve vertices of icosahedron on unit sphere
        float tau = 0.8506508084f;
        float one = 0.5257311121f;

        Eigen::Vector3f v[12];

        setVec(v[0], tau, one, 0);
        setVec(v[1], -tau, one, 0);
        setVec(v[2], -tau, -one, 0);
        setVec(v[3], tau, -one, 0);
        setVec(v[4], one, 0, tau);
        setVec(v[5], one, 0, -tau);
        setVec(v[6], -one, 0, -tau);
        setVec(v[7], -one, 0, tau);
        setVec(v[8], 0, tau, one);
        setVec(v[9], 0, -tau, one);
        setVec(v[10], 0, -tau, -one);
        setVec(v[11], 0, tau, -one);

        for (int i = 0 ; i < 12 ; i++)
        {
            gd.vertices.push_back(v[i]);
        }


        // Structure for unit icosahedron
        MathTools::TriangleFace f[20];
        f[0].set(5,  8,  9);
        f[1].set(5, 10,  8);
        f[2].set(6, 12,  7);
        f[3].set(6,  7, 11);
        f[4].set(1,  4,  5);
        f[5].set(1,  6,  4);
        f[6].set(3,  2,  8);
        f[7].set(3,  7,  2);
        f[8].set(9, 12,  1);
        f[9].set(9,  2, 12);
        f[10].set(10,  4, 11);
        f[11].set(10, 11, 3);
        f[12].set(9,  1,  5);
        f[13].set(12,  6,  1);
        f[14].set(5,  4, 10);
        f[15].set(6, 11,  4);
        f[16].set(8,  2,  9);
        f[17].set(7, 12,  2);
        f[18].set(8, 10, 3);
        f[19].set(7,  3, 11);

        for (int i = 0 ; i < 20 ; i++)
        {
            f[i].id1--;
            f[i].id2--;
            f[i].id3--;
            gd.faces.push_back(f[i]);
        }
    }

    void SphereApproximator::subDivide(GraphData& gd)
    {
        // buffer new faces
        vector<MathTools::TriangleFace> newFaces;

        // gp through all faces and subdivide
        for (int f = 0 ; f < int(gd.faces.size()) ; f++)
        {
            // get the triangle vertex indices
            int nA = gd.faces.at(f).id1;
            int nB = gd.faces.at(f).id2;
            int nC = gd.faces.at(f).id3;

            // get the triangle vertex coordinates
            Eigen::Vector3f a = gd.vertices.at(nA);
            Eigen::Vector3f b = gd.vertices.at(nB);
            Eigen::Vector3f c = gd.vertices.at(nC);

            // Now find the midpoints between vertices
            Eigen::Vector3f a_m, b_m, c_m;
            addVecVec(a, b, a_m);
            mulVecScalar(a_m, 0.5, a_m);
            addVecVec(b, c, b_m);
            mulVecScalar(b_m, 0.5, b_m);
            addVecVec(c, a, c_m);
            mulVecScalar(c_m, 0.5, c_m);
            /*      printf("a: %f,%f,%f\n", a(0),a(1),a(2));
                    printf("b: %f,%f,%f\n", b(0),b(1),b(2));
                    printf("c: %f,%f,%f\n", c(0),c(1),c(2));

                    printf("a<->b: %f,%f,%f\n", a_m(0),a_m(1),a_m(2));
                    printf("b<->c: %f,%f,%f\n", b_m(0),b_m(1),b_m(2));
                    printf("c<->a: %f,%f,%f\n", c_m(0),c_m(1),c_m(2));*/

            // add vertices
            int nA_m, nB_m, nC_m;

            int nIndex;
            nIndex = findVertex(a_m, 0.01f, gd.vertices);

            if (nIndex == -1)
            {
                gd.vertices.push_back(a_m);
                nA_m = (int)gd.vertices.size() - 1;
            }
            else
            {
                nA_m = nIndex;
            }

            nIndex = findVertex(b_m, 0.01f, gd.vertices);

            if (nIndex == -1)
            {
                gd.vertices.push_back(b_m);
                nB_m = (int)gd.vertices.size() - 1;
            }
            else
            {
                nB_m = nIndex;
            }

            nIndex = findVertex(c_m, 0.01f, gd.vertices);

            if (nIndex == -1)
            {
                gd.vertices.push_back(c_m);
                nC_m = (int)gd.vertices.size() - 1;
            }
            else
            {
                nC_m = nIndex;
            }

            // Create new faces with orig vertices plus midpoints
            MathTools::TriangleFace fa[4];
            fa[0].set(nA, nA_m, nC_m);
            fa[1].set(nA_m, nB, nB_m);
            fa[2].set(nC_m, nB_m, nC);
            fa[3].set(nA_m, nB_m, nC_m);

            for (int i = 0 ; i < 4 ; i++)
            {
                newFaces.push_back(fa[i]);
            }
        }

        // assign new faces
        gd.faces = newFaces;
    }


    void SphereApproximator::addVecVec(const Eigen::Vector3f& vector1, const Eigen::Vector3f& vector2, Eigen::Vector3f& result)
    {
        result = vector1 + vector2;
    }

    void SphereApproximator::mulVecScalar(const Eigen::Vector3f& vec, float scalar, Eigen::Vector3f& result)
    {
        result = scalar * vec;
    }

    void SphereApproximator::projectToSphere(float radius, GraphData& gd)
    {
        for (int v = 0 ; v < int(gd.vertices.size()) ; v++)
        {
            float X = (float)gd.vertices.at(v)(0);
            float Y = (float)gd.vertices.at(v)(1);
            float Z = (float)gd.vertices.at(v)(2);

            float x, y, z;

            // Convert Cartesian X,Y,Z to spherical (radians)
            float theta = atan2(Y, X);
            float phi   = atan2(sqrt(X * X + Y * Y), Z);

            // Recalculate X,Y,Z for constant r, given theta & phi.
            x = radius * sin(phi) * cos(theta);
            y = radius * sin(phi) * sin(theta);
            z = radius * cos(phi);

            gd.vertices.at(v)(0) = x;
            gd.vertices.at(v)(1) = y;
            gd.vertices.at(v)(2) = z;
        }
    }

    float SphereApproximator::AngleVecVec(const Eigen::Vector3f& vector1, const Eigen::Vector3f& vector2)
    {
        const float sp = (float)(vector1(0) * vector2(0) + vector1(1) * vector2(1) + vector1(2) * vector2(2));
        const float l1 = sqrtf((float)(vector1(0) * vector1(0) + vector1(1) * vector1(1) + vector1(2) * vector1(2)));
        const float l2 = sqrtf((float)(vector2(0) * vector2(0) + vector2(1) * vector2(1) + vector2(2) * vector2(2)));


        // added this. In some cases angle was numerically unstable
        if (fabs(l1 * l2) < 1e-12)
        {
            return 0.0f;
        }

        float r = sp / (l1 * l2);

        if (r > 1.0)
        {
            r = 1.0;
        }

        if (r < -1.0)
        {
            r = -1.0;
        }

        return acosf(r);
    }

    int SphereApproximator::findVertex(const Eigen::Vector3f& position, float epsilon, std::vector<Eigen::Vector3f>& vertices)
    {
        float fDistance;
        float fMinDistance = FLT_MAX;
        int nIndex = -1;

        for (int v = 0 ; v < int(vertices.size()) ; v++)
        {
            fDistance = AngleVecVec(position, vertices.at(v));

            if (fDistance < fMinDistance)
            {
                fMinDistance = fDistance;
                nIndex = v;
            }
        }

        if (fMinDistance > epsilon)
        {
            nIndex = -1;
        }

        return nIndex;
    }

    /*
    SoSeparator* SphereApproximator::generateIVModel(SphereApproximation &sphereApprox, float fTransparency)
    {
        SoSeparator* pResult = new SoSeparator();
        SoCoordinate3* pCoords = new SoCoordinate3();
        //SoFaceSet* pFaceSet = new SoFaceSet();

        if (sphereApprox.m_pVisualization)
        {
            cout << __FUNCTION__ << ": Warning: already built a visu, overriding the visualization points..." << endl;
        }

        Eigen::Vector3f v1,v2,v3;
        Face f;

        int nVertices = (int)sphereApprox.vertices.size();
        int nFaces = (int)sphereApprox.faces.size();
        if (nVertices<=0)
            return pResult;
        const int nVertReal = nFaces*3;
        SbVec3f *pVertexArray = new SbVec3f[nVertReal];

        int nVertexCount = 0;

        long *nNumVertices = new long[4];
        nNumVertices[3] = -1;

        SoSeparator *pFaceSeperator = new SoSeparator();

        for (int i=0;i<nFaces;i++)
        {
            f = sphereApprox.faces.at(i);
            v1 = sphereApprox.vertices.at(f.id1);
            v2 = sphereApprox.vertices.at(f.id2);
            v3 = sphereApprox.vertices.at(f.id3);
            SoSeparator *pFaceSep = new SoSeparator();
            SoMaterial* pFaceMat = new SoMaterial();
            pFaceMat->transparency.setValue(fTransparency);
            SoIndexedFaceSet* pFaceSet1 = new SoIndexedFaceSet();
            nNumVertices[0] = i*3;
            nNumVertices[1] = i*3+1;
            nNumVertices[2] = i*3+2;
            nNumVertices[3] = -1;
            pFaceSet1->coordIndex.setValues(0,4,(const int32_t*)nNumVertices);
            pFaceSep->addChild(pFaceMat);
            pFaceSep->addChild(pFaceSet1);

            sphereApprox.faces.at(i).m_pVisualization = pFaceSep;

            pFaceSeperator->addChild(pFaceSep);

            // COUNTER CLOCKWISE
            pVertexArray[nVertexCount].setValue((float)v3(0),(float)v3(1),(float)v3(2));
            nVertexCount++;
            pVertexArray[nVertexCount].setValue((float)v2(0),(float)v2(1),(float)v2(2));
            nVertexCount++;
            pVertexArray[nVertexCount].setValue((float)v1(0),(float)v1(1),(float)v1(2));
            nVertexCount++;
        }
        pCoords->point.setValues(0,nVertReal,pVertexArray);
        //long *nNumVertices = new long[nFaces];
        for (int i=0;i<nFaces;i++)
            nNumVertices[i] = 3;
        pFaceSet->numVertices.setValues(0,nFaces,(const int32_t*)nNumVertices);
     
        pResult->addChild(pCoords);
        pResult->addChild(pFaceSeperator);
        delete []pVertexArray;
        delete []nNumVertices;
        sphereApprox.m_pVisualization = pResult;
        return pResult;
    }*/

    bool SphereApproximator::check_same_clock_dir(const Eigen::Vector3f& pt1, const Eigen::Vector3f& pt2, const Eigen::Vector3f& pt3, const Eigen::Vector3f& norm)
    {
        float testi, testj, testk;
        float dotprod;
        // normal of triangle
        testi = (((pt2(1) - pt1(1)) * (pt3(2) - pt1(2))) - ((pt3(1) - pt1(1)) * (pt2(2) - pt1(2))));
        testj = (((pt2(2) - pt1(2)) * (pt3(0) - pt1(0))) - ((pt3(2) - pt1(2)) * (pt2(0) - pt1(0))));
        testk = (((pt2(0) - pt1(0)) * (pt3(1) - pt1(1))) - ((pt3(0) - pt1(0)) * (pt2(1) - pt1(1))));

        // Dot product with triangle normal
        dotprod = testi * norm(0) + testj * norm(1) + testk * norm(2);

        //answer
        if (dotprod < 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    bool SphereApproximator::check_intersect_tri(const Eigen::Vector3f& pt1, const Eigen::Vector3f& pt2, const Eigen::Vector3f& pt3, const Eigen::Vector3f& linept, const Eigen::Vector3f& vect, Eigen::Vector3f& storeIntersection)
    {
        // code from http://www.angelfire.com/fl/houseofbartlett/solutions/line2tri.html
        Eigen::Vector3f v1, v2, norm;
        float dotprod;
        float t;

        // vector form triangle pt1 to pt2
        v1 = pt2 - pt1;


        // vector form triangle pt2 to pt3
        v2 = pt3 - pt2;

        // vector normal of triangle
        norm = v1.cross(v2);

        // dot product of normal and line's vector if zero line is parallel to triangle
        dotprod = norm.dot(vect);
        // norm(0)*vect(0) + norm(1)*vect(1) + norm(2)*vect(2);

        if (dotprod < 0)
        {
            //Find point of intersect to triangle plane.
            //find t to intersect point
            t = -(norm(0) * (linept(0) - pt1(0)) + norm(1) * (linept(1) - pt1(1)) + norm(2) * (linept(2) - pt1(2))) /
                (norm(0) * vect(0) + norm(1) * vect(1) + norm(2) * vect(2));

            // if ds is neg line started past triangle so can't hit triangle.
            if (t < 0)
            {
                return false;
            }

            storeIntersection = linept + vect * t;

            if (check_same_clock_dir(pt1, pt2, storeIntersection, norm))
            {
                if (check_same_clock_dir(pt2, pt3, storeIntersection, norm))
                {
                    if (check_same_clock_dir(pt3, pt1, storeIntersection, norm))
                    {
                        // answer in pt_int is inside triangle
                        return true;
                    }
                }
            }
        }

        return false;
    }

    VirtualRobot::TriMeshModelPtr SphereApproximator::generateTriMesh(const SphereApproximation& a)
    {
        Eigen::Vector3f v1, v2, v3;
        MathTools::TriangleFace f;

        int nVertices = (int)a.vertices.size();
        int nFaces = (int)a.faces.size();
        VR_ASSERT(nVertices > 0);
        int nVertexCount = 0;

        TriMeshModelPtr tr(new TriMeshModel());

        for (int i = 0; i < nFaces; i++)
        {
            f = a.faces.at(i);
            v1 = a.vertices.at(f.id1);
            v2 = a.vertices.at(f.id2);
            v3 = a.vertices.at(f.id3);
            tr->addTriangleWithFace(v1, v3, v2);
        }

        return tr;
    }

}

