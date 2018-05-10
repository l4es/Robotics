
#include "ConvexHullGenerator.h"
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <math.h>
#include <iostream>
#include <float.h>

//#define CONVEXHULL_DEBUG_OUTPUT

using namespace std;
using namespace VirtualRobot;
using namespace VirtualRobot::MathTools;

extern "C"
{
#include "ExternalDependencies/qhull-2003.1/include/qhull/qhull_a.h"
}
namespace GraspStudio
{
    boost::mutex ConvexHullGenerator::qhull_mutex;

    bool ConvexHullGenerator::ConvertPoints(std::vector<Eigen::Vector3f>& points, double* storePointsQHull, bool lockMutex)
    {
        if (lockMutex)
        {
            qhull_mutex.lock();
        }

        for (int i = 0; i < (int)points.size(); i++)
        {
            storePointsQHull[i * 3 + 0] = points[i][0];
            storePointsQHull[i * 3 + 1] = points[i][1];
            storePointsQHull[i * 3 + 2] = points[i][2];
        }

        if (lockMutex)
        {
            qhull_mutex.unlock();
        }

        return true;
    }

    bool ConvexHullGenerator::ConvertPoints(std::vector<ContactPoint>& points, double* storePointsQHull, bool lockMutex)
    {
        if (lockMutex)
        {
            qhull_mutex.lock();
        }

        for (int i = 0; i < (int)points.size(); i++)
        {
            storePointsQHull[i * 6 + 0] = points[i].p[0];
            storePointsQHull[i * 6 + 1] = points[i].p[1];
            storePointsQHull[i * 6 + 2] = points[i].p[2];
            storePointsQHull[i * 6 + 3] = points[i].n[0];
            storePointsQHull[i * 6 + 4] = points[i].n[1];
            storePointsQHull[i * 6 + 5] = points[i].n[2];
        }

        if (lockMutex)
        {
            qhull_mutex.unlock();
        }

        return true;
    }


    VirtualRobot::MathTools::ConvexHull3DPtr ConvexHullGenerator::CreateConvexHull(VirtualRobot::TriMeshModelPtr pointsInput, bool lockMutex /*= true*/)
    {
        if (lockMutex)
        {
            qhull_mutex.lock();
        }

        VirtualRobot::MathTools::ConvexHull3DPtr r = CreateConvexHull(pointsInput->vertices, false);

        if (lockMutex)
        {
            qhull_mutex.unlock();
        }

        return r;
    }



    VirtualRobot::MathTools::ConvexHull3DPtr ConvexHullGenerator::CreateConvexHull(std::vector<Eigen::Vector3f>& pointsInput, bool lockMutex /*= true*/)
    {
        if (lockMutex)
        {
            qhull_mutex.lock();
        }

        clock_t startT = clock();

        ConvexHull3DPtr result(new ConvexHull3D());
        result->faces.clear();
        result->vertices.clear();
        int nPoints = (int)pointsInput.size();

        if (nPoints < 4)
        {
            cout << __FUNCTION__ << "Error: Need at least 4 points (nr of points registered: " << nPoints << ")" << endl;

            if (lockMutex)
            {
                qhull_mutex.unlock();
            }

            return result;
        }

        int dim = 3;                /* dimension of points */
        int numpoints = nPoints;    /* number of points */
        coordT* points = new coordT[(3)*nPoints];   /* array of coordinates for each point */
        boolT ismalloc = False;     /* True if qhull should free points in qh_freeqhull() or reallocation */
        char flags[250];            /* option flags for qhull, see qh_opt.htm */
#ifdef CONVEXHULL_DEBUG_OUTPUT
        FILE* outfile = stdout;     /* output from qh_produce_output()*/
        /* use NULL to skip qh_produce_output() */
# else
        FILE* outfile = NULL;       /* output from qh_produce_output()*/
        /* use NULL to skip qh_produce_output() */
#endif
        FILE* errfile = stderr;     /* error messages from qhull code */
        int exitcode;               /* 0 if no error from qhull */
        facetT* facet;              /* set by FORALLfacets */
        int curlong, totlong;       /* memory remaining after qh_memfreeshort */
        vertexT* vertex, **vertexp;

        // qhull options:
        // Tv: test result
        // Tcv: test result
        // Qt: triangulate result (not really?!)
        // QJ: juggling input points, triangulated result
        // FA: compute volume
#ifdef CONVEXHULL_DEBUG_OUTPUT
        sprintf(flags, "qhull s Tcv Qt FA");
#else
        sprintf(flags, "qhull s Qt FA");  // QJ is faster than Qt (but the results seem to be less accurate, see www.qhull.org documentation)
#endif

        //cout << "QHULL input: nVertices: " << pointsInput.size() << endl;
        ConvertPoints(pointsInput, points, false);
        /*for (i=numpoints; i--; )
        rows[i]= points+dim*i;
        qh_printmatrix (outfile, "input", rows, numpoints, dim);*/
        exitcode = qh_new_qhull(dim, numpoints, points, ismalloc, flags, outfile, errfile);

        if (!exitcode)                    /* if no error */
        {
            facetT* facet_list = qh facet_list;
            int convexNumFaces = qh num_facets;
            int convexNumVert = qh_setsize(qh_facetvertices(facet_list, NULL, false));

            qh_triangulate(); // need this for triangulated output!
            int convexNumFaces2 = qh num_facets;
            int convexNumVert2 = qh_setsize(qh_facetvertices(facet_list, NULL, false));
            /*
            cout << "Numfacets1:" << convexNumFaces << endl;
            cout << "Numvertices1:" << convexNumVert << endl;
            cout << "Numfacets2:" << convexNumFaces2 << endl;
            cout << "Numvertices2:" << convexNumVert2 << endl;*/
            /* 'qh facet_list' contains the convex hull */
            Eigen::Vector3f v[3];
            int nIds[3];
            TriangleFace f;

            int nFacets = 0;
            //cout << "Volume: " << qh totvol << endl;
            qh_getarea(qh facet_list);
            //cout << "Volume: " << qh totvol << endl;
            result->volume = qh totvol;


            double pCenter[3];

            for (int u = 0; u < 3; u++)
            {
                pCenter[u] = 0;
            }

            int nVcertexCount = 0;
            FORALLvertices
            {
                for (int u = 0; u < 3; u++)
                {
                    pCenter[u] += vertex->point[u];
                }

                nVcertexCount++;
            }

            if (nVcertexCount > 0)
                for (int u = 0; u < 3; u++)
                {
                    pCenter[u] /= (float)nVcertexCount;
                }

            result->center[0] = pCenter[0];
            result->center[1] = pCenter[1];
            result->center[2] = pCenter[2];

            float maxDist = 0;

            FORALLfacets
            {
                
                int c = 0;
#ifdef CONVEXHULL_DEBUG_OUTPUT
                cout << "FACET " << nFacets << ":" << endl;
#endif

                FOREACHvertex_(facet->vertices)
                {
#ifdef CONVEXHULL_DEBUG_OUTPUT
                    cout << vertex->point[0] << "," << vertex->point[1] << "," << vertex->point[2] << endl;
#endif

                    if (c < 3)
                    {
                        v[c][0] = vertex->point[0];
                        v[c][1] = vertex->point[1];
                        v[c][2] = vertex->point[2];
                        result->vertices.push_back(v[c]);
                        nIds[c] = (int)result->vertices.size() - 1;
                        c++;
                    }
                    else
                    {
                        cout << __FUNCTION__ << ": Error, facet with more than 3 vertices not supported ... face nr:" << nFacets << endl;
                    }
                }
                f.id1 = nIds[0];
                f.id2 = nIds[1];
                f.id3 = nIds[2];
                f.normal[0] = facet->normal[0];
                f.normal[1] = facet->normal[1];
                f.normal[2] = facet->normal[2];
                //cout << "Normal: " << f.m_Normal.x << "," << f.m_Normal.y << "," << f.m_Normal.z << endl;

                double dist = qh_distnorm(3, pCenter, facet->normal, &(facet->offset));
                if (fabs(dist) > maxDist)
                    maxDist = fabs(dist);


                result->faces.push_back(f);
                nFacets++;
            }
            result->maxDistFacetCenter = maxDist;
            /*
            cout << "QHULL result: nVertices: " << storeResult.vertices.size() << endl;
            cout << "QHULL result: nFactes: " << nFacets << endl;
            */
        }


        qh_freeqhull(!qh_ALL);
        qh_memfreeshort(&curlong, &totlong);

        if (curlong || totlong)
            fprintf(errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
                    totlong, curlong);

        clock_t endT = clock();
        long timeMS = (long)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0);

        //cout << __FUNCTION__ << ": Created convex hull in " << timeMS << " ms" << endl;
        if (lockMutex)
        {
            qhull_mutex.unlock();
        }

        return result;
    }

    VirtualRobot::MathTools::ConvexHull6DPtr ConvexHullGenerator::CreateConvexHull(std::vector<ContactPoint>& pointsInput, bool lockMutex)
    {
        if (lockMutex)
        {
            qhull_mutex.lock();
        }

        clock_t startT = clock();

        ConvexHull6DPtr result(new ConvexHull6D());

        int nPoints = (int)pointsInput.size();

        if (nPoints < 4)
        {
            cout << __FUNCTION__ << "Error: Need at least 4 points (nr of points registered: " << nPoints << ")" << endl;

            if (lockMutex)
            {
                qhull_mutex.unlock();
            }

            return result;
        }

        int dim = 6;                /* dimension of points */
        int numpoints = nPoints;    /* number of points */
        coordT* points = new coordT[(6)*nPoints];   /* array of coordinates for each point */
        boolT ismalloc = False;     /* True if qhull should free points in qh_freeqhull() or reallocation */
        char flags[250];            /* option flags for qhull, see qh_opt.htm */
#ifdef CONVEXHULL_DEBUG_OUTPUT
        FILE* outfile = stdout;     /* output from qh_produce_output()*/
        /* use NULL to skip qh_produce_output() */
# else
        FILE* outfile = NULL;       /* output from qh_produce_output()*/
        /* use NULL to skip qh_produce_output() */
#endif
        FILE* errfile = stderr;     /* error messages from qhull code */
        int exitcode;               /* 0 if no error from qhull */
        facetT* facet;              /* set by FORALLfacets */
        int curlong, totlong;       /* memory remaining after qh_memfreeshort */
        vertexT* vertex, **vertexp;

        // qhull options:
        // Tv: test result
        // Tcv: test result
        // Qt: triangulated result? -> no, we need to triangulate the result with qh_triangulate()
        // QJ: juggling input points, triangulated result (avoids errors with coplanar facets etc)
        // FA: compute volume
#ifdef CONVEXHULL_DEBUG_OUTPUT
        //sprintf (flags, "qhull s Tcv Qt FA");
        sprintf(flags, "qhull s QJ FA");
        std::cout.precision(2);
#else
        //sprintf (flags, "qhull s Qt FA"); // some of the resulting facets are not triangulated ?!
        sprintf(flags, "qhull QJ FA");  // QJ is faster than Qt (but the results seem to be less accurate, see www.qhull.org documentation)
#endif

        //cout << "QHULL input: nVertices: " << pointsInput.size() << endl;
        //printVertices(pointsInput);
        ConvertPoints(pointsInput, points, false);
        exitcode = qh_new_qhull(dim, numpoints, points, ismalloc,
                                flags, outfile, errfile);

        if (!exitcode)                    /* if no error */
        {
            facetT* facet_list = qh facet_list;
            int convexNumFaces = qh num_facets;
            int convexNumVert = qh_setsize(qh_facetvertices(facet_list, NULL, false));

            qh_triangulate(); // need this for triangulated output!
            int convexNumFaces2 = qh num_facets;
            int convexNumVert2 = qh_setsize(qh_facetvertices(facet_list, NULL, false));
            double pCenter[6];

            for (int u = 0; u < 6; u++)
            {
                pCenter[u] = 0;
            }

            double pZero[6];

            for (int u = 0; u < 6; u++)
            {
                pZero[u] = 0;
            }

            int nVcertexCount = 0;
            FORALLvertices
            {
                for (int u = 0; u < 6; u++)
                {
                    pCenter[u] += vertex->point[u];
                }

                nVcertexCount++;
            }

            if (nVcertexCount > 0)
                for (int u = 0; u < 6; u++)
                {
                    pCenter[u] /= (float)nVcertexCount;
                }

            result->center.p[0] = pCenter[0];
            result->center.p[1] = pCenter[1];
            result->center.p[2] = pCenter[2];
            result->center.n[0] = pCenter[3];
            result->center.n[1] = pCenter[4];
            result->center.n[2] = pCenter[5];

            /* 'qh facet_list' contains the convex hull */
            ContactPoint v[6];
            int nIds[6];
            MathTools::TriangleFace6D f;

            int nFacets = 0;
            qh_getarea(qh facet_list);
            result->volume = qh totvol;
            double p[6];
            p[0] = p[1] = p[2] = p[3] = p[4] = p[5] = 0;
            FORALLfacets
            {
                int c = 0;
#ifdef CONVEXHULL_DEBUG_OUTPUT
                bool printInfo = false;

                if (facet->offset > 0)
                {
                    printInfo = true;
                }

                if (printInfo)
                {
                    cout << "FACET " << nFacets << ":" << endl;
                    cout << "Offset:" << facet->offset << endl;
                }

#endif

                FOREACHvertex_(facet->vertices)
                {
#ifdef CONVEXHULL_DEBUG_OUTPUT

                    if (printInfo)
                    {
                        cout << vertex->point[0] << "," << vertex->point[1] << "," << vertex->point[2] << "," << vertex->point[3] << "," << vertex->point[4] << "," << vertex->point[5] << endl;
                    }

#endif

                    if (c < 6)
                    {
                        v[c].p[0] = vertex->point[0];
                        v[c].p[1] = vertex->point[1];
                        v[c].p[2] = vertex->point[2];
                        v[c].n[0] = vertex->point[3];
                        v[c].n[1] = vertex->point[4];
                        v[c].n[2] = vertex->point[5];
                        result->vertices.push_back(v[c]);
                        nIds[c] = (int)result->vertices.size() - 1;
                        c++;
                    }
                    else
                    {
                        cout << __FUNCTION__ << ": Error, facet with more than 6 vertices not supported ... face nr:" << nFacets << endl;
                    }
                }
                f.id[0] = nIds[0];
                f.id[1] = nIds[1];
                f.id[2] = nIds[2];
                f.id[3] = nIds[3];
                f.id[4] = nIds[4];
                f.id[5] = nIds[5];
                f.normal.p[0] = facet->normal[0];
                f.normal.p[1] = facet->normal[1];
                f.normal.p[2] = facet->normal[2];
                f.normal.n[0] = facet->normal[3];
                f.normal.n[1] = facet->normal[4];
                f.normal.n[2] = facet->normal[5];
                f.offset = facet->offset;
                double dist = qh_distnorm(6, pCenter, facet->normal, &(facet->offset));
                f.distNormCenter = dist;
                qh_distplane(pCenter, facet, &dist);
                f.distPlaneCenter = dist;
                dist = qh_distnorm(6, pZero, facet->normal, &(facet->offset));
                f.distNormZero = dist;
                qh_distplane(pZero, facet, &dist);
                f.distPlaneZero = dist;
#ifdef CONVEXHULL_DEBUG_OUTPUT

                if (printInfo)
                {

                    if (facet->center)
                    {
                        cout << "Center:" << facet->center[0] << "," << facet->center[1] << "," << facet->center[2] << "," << facet->center[3] << "," << facet->center[4] << "," << facet->center[5] << endl;
                    }

                    cout << "distPlaneZero: " << f.distPlaneZero << ", distNormZero:" << f.distNormZero << ", QHULL_OFFSET:" << facet->offset << endl;
                    //cout << "Normal: " << f.normal.x << "," << f.normal.y << "," << f.normal.z << "," << f.normal.nx << "," << f.normal.ny << "," << f.normal.nz << endl;
                }

#endif
                result->faces.push_back(f);
                nFacets++;
            }
            /*cout << "QHULL result: created Vertices: " << storeResult.vertices.size() << endl;
            cout << "QHULL result: created Faces: " << nFacets << endl;
            cout << "QHULL result: nVertices: " << convexNumVert2 << endl;
            cout << "QHULL result: nFactes: " << convexNumFaces2 << endl;*/
        }

        qh_freeqhull(!qh_ALL);
        qh_memfreeshort(&curlong, &totlong);

        if (curlong || totlong)
            fprintf(errfile, "qhull internal warning (main): did not free %d bytes of long memory (%d pieces)\n",
                    totlong, curlong);

        clock_t endT = clock();
        long timeMS = (long)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0);

        //cout << __FUNCTION__ << ": Created 6D convex hull in " << timeMS << " ms" << endl;
        if (lockMutex)
        {
            qhull_mutex.unlock();
        }

        return result;
    }

    /*
    bool createPoints( SoSeparator *pInputIVModel, std::vector<Vec3D> &vStorePoints, bool lockMutex )
    {
        if (!pInputIVModel)
            return false;

        if (lockMutex)
            qhull_mutex.lock();
        vStorePoints.clear();
        SoCallbackAction ca;
        ca.addTriangleCallback(SoShape::getClassTypeId(), &CConvexHullGenerator_triangleCB, &vStorePoints);
        ca.apply(pInputIVModel);
        if (lockMutex)
            qhull_mutex.unlock();
        return true;
    }*/

    /*
    bool createConvexHull(SoSeparator *pInputIVModel, ConvexHull3D &storeResult)
    {
        qhull_mutex.lock();
        vector<Vec3D> points;
        if (!CreatePoints(pInputIVModel,points,false))
            return false;
        bool bRes = CreateConvexHull(points,storeResult,false);
        qhull_mutex.unlock();
        return bRes;
    }*/

    /*
    bool createIVModel( ConvexHull3D &convexHull, SoSeparator *pStoreResult, bool lockMutex )
    {
        if (!pStoreResult || convexHull.vertices.size()<=0 || convexHull.faces.size()<=0)
            return false;

        if (lockMutex)
            qhull_mutex.lock();

        SoCoordinate3* pCoords = new SoCoordinate3();
        SoFaceSet* pFaceSet = new SoFaceSet();

        int nFaces = (int)convexHull.faces.size();
        int nVertices = nFaces*3;
        Face3D f;
        Vec3D v1,v2,v3;

        SbVec3f *pVertexArray = new SbVec3f[nVertices];

        int nVertexCount = 0;

        for (int i=0;i<nFaces;i++)
        {
            f = convexHull.faces.at(i);
            v1 = convexHull.vertices.at(f.id[0]);
            v2 = convexHull.vertices.at(f.id[1]);
            v3 = convexHull.vertices.at(f.id[2]);

            bool bNeedFlip = GraspStudioHelpers::checkVerticeOrientation(v1,v2,v3,f.normal);

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

        }
        pCoords->point.setValues(0,nVertices,pVertexArray);
        long *nNumVertices = new long[nFaces];
        for (int i=0;i<nFaces;i++)
            nNumVertices[i] = 3;
        pFaceSet->numVertices.setValues(0,nFaces,(const int32_t*)nNumVertices);

        pStoreResult->addChild(pCoords);
        pStoreResult->addChild(pFaceSet);
        delete []pVertexArray;
        delete []nNumVertices;

        if (lockMutex)
            qhull_mutex.unlock();

        return true;
    }*/

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

    /*
    bool createIVModel(ConvexHull6D &convHull, SoSeparator *pStoreResult, bool buseFirst3Coords)
    {
        if (!pStoreResult || convHull.vertices.size()<=0 || convHull.faces.size()<=0)
            return false;

        qhull_mutex.lock();
        Face6D f;
        Vec3D v1,v2,v3,v4,v5,v6;


        int nFaces = (int)convHull.faces.size();


        // project points to 3d, then create hull of these points to visualize it
        std::vector<Vec3D> vProjectedPoints;
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
            }
            vProjectedPoints.push_back(v1);
            vProjectedPoints.push_back(v2);
            vProjectedPoints.push_back(v3);
            vProjectedPoints.push_back(v4);
            vProjectedPoints.push_back(v5);
            vProjectedPoints.push_back(v6);
        }
        ConvexHull3D projectedHull;
        if (!CreateConvexHull(vProjectedPoints, projectedHull, false))
        {
            cout << __FUNCTION__ << " Could not create hull of projected points, aborting..." << endl;
            qhull_mutex.unlock();
            return false;
        }
        bool bRes = CreateIVModel(projectedHull, pStoreResult, false);
        qhull_mutex.unlock();
        return bRes;
        / *
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
        * /
    }*/

    void ConvexHullGenerator::PrintVertices(std::vector<ContactPoint>& pointsInput)
    {
        for (int i = 0; i < (int)pointsInput.size(); i++)
        {
            cout << "v" << i << ": " << pointsInput[i].p[0] << "," << pointsInput[i].p[1] << "," << pointsInput[i].p[2] << ","
                 << pointsInput[i].n[0] << "," << pointsInput[i].n[1] << "," << pointsInput[i].n[2] << endl;
        }
    }
    void ConvexHullGenerator::PrintStatistics(VirtualRobot::MathTools::ConvexHull6DPtr convHull)
    {
        if (!convHull)
        {
            GRASPSTUDIO_ERROR << " Null data to print" << endl;
            return;
        }

        float minValue[6];
        float maxValue[6];
        int i;

        for (i = 0; i <= 5; i++)
        {
            minValue[i] = FLT_MAX;
            maxValue[i] = -FLT_MAX;
        }

        for (i = 0; i < (int)convHull->vertices.size(); i++)
        {
            if (convHull->vertices[i].p[0] < minValue[0])
            {
                minValue[0] = convHull->vertices[i].p[0];
            }

            if (convHull->vertices[i].p[0] > maxValue[0])
            {
                maxValue[0] = convHull->vertices[i].p[0];
            }

            if (convHull->vertices[i].p[1] < minValue[1])
            {
                minValue[1] = convHull->vertices[i].p[1];
            }

            if (convHull->vertices[i].p[1] > maxValue[1])
            {
                maxValue[1] = convHull->vertices[i].p[1];
            }

            if (convHull->vertices[i].p[2] < minValue[2])
            {
                minValue[2] = convHull->vertices[i].p[2];
            }

            if (convHull->vertices[i].p[2] > maxValue[2])
            {
                maxValue[2] = convHull->vertices[i].p[2];
            }

            if (convHull->vertices[i].n[0] < minValue[3])
            {
                minValue[3] = convHull->vertices[i].n[0];
            }

            if (convHull->vertices[i].n[0] > maxValue[3])
            {
                maxValue[3] = convHull->vertices[i].n[0];
            }

            if (convHull->vertices[i].n[1] < minValue[4])
            {
                minValue[4] = convHull->vertices[i].n[1];
            }

            if (convHull->vertices[i].n[1] > maxValue[4])
            {
                maxValue[4] = convHull->vertices[i].n[1];
            }

            if (convHull->vertices[i].n[2] < minValue[5])
            {
                minValue[5] = convHull->vertices[i].n[2];
            }

            if (convHull->vertices[i].n[2] > maxValue[5])
            {
                maxValue[5] = convHull->vertices[i].n[2];
            }
        }

        cout << "Conv Hull Bounds:" << endl;
        cout << "\t\t x : " << minValue[0] << "," << maxValue[0] << endl;
        cout << "\t\t y : " << minValue[1] << "," << maxValue[1] << endl;
        cout << "\t\t z : " << minValue[2] << "," << maxValue[2] << endl;
        cout << "\t\t nx: " << minValue[3] << "," << maxValue[3] << endl;
        cout << "\t\t ny: " << minValue[4] << "," << maxValue[4] << endl;
        cout << "\t\t nz: " << minValue[5] << "," << maxValue[5] << endl;
    }

    bool ConvexHullGenerator::checkVerticeOrientation(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3, const Eigen::Vector3f& n)
    {
        Eigen::Vector3f tmp;
        Eigen::Vector3f v1v2;
        Eigen::Vector3f v1v3;
        v1v2(0) = v2(0) - v1(0);
        v1v2(1) = v2(1) - v1(1);
        v1v2(2) = v2(2) - v1(2);
        v1v3(0) = v3(0) - v1(0);
        v1v3(1) = v3(1) - v1(1);
        v1v3(2) = v3(2) - v1(2);
        tmp = v1v2.cross(v1v3);
        float tmpF = tmp.dot(n);
        return (tmpF < 0);
        /*crossProduct(v1v2,v1v3,tmp);
        float tmpF = dotProduct(tmp,n);*/
    }



} // namespace
