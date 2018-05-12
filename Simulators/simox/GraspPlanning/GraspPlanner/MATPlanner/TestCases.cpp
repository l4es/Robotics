#include "TestCases.h"

using namespace std;
namespace GraspStudio
{

    TestCases::TestCases()
    {
    }

    void TestCases::testScaleMedialSphere()
    {
        cout << "\n---> tcScaleMedialSphere()" << endl;

        Eigen::Vector3f center(0, 2, 0);
        float sphereRadius = 1.0f;
        Eigen::Vector3f p1(-1, 2, 0);
        Eigen::Vector3f p2(0, 1, 0);
        Eigen::Vector3f p3(1, 2, 0);
        Eigen::Vector3f p4(0, 3, 0);
        vector<Eigen::Vector3f> surfacePoints;

        surfacePoints.push_back(p1);
        surfacePoints.push_back(p2);
        surfacePoints.push_back(p3);
        surfacePoints.push_back(p4);
        MedialSphere ms(center, sphereRadius, surfacePoints);

        cout << "MedialSphere BEFORE Scaling:" << std::endl;
        ms.printDebug();

        //Scaling:
        float scaleFactor = 1000.0f;

        cout << "\nScaling with factor " << scaleFactor << " ..." << std::endl;
        ms.scale(scaleFactor);

        cout << "MedialSphere AFTER Scaling:" << std::endl;
        ms.printDebug();
        cout << endl;

    }

    void TestCases::testScaleMedialSphere(MedialSpherePtr ms, float scaleFactor)
    {
        cout << "\n---> testScaleMedialSphere(ms)" << endl;

        cout << "MedialSphere BEFORE Scaling:" << std::endl;
        ms->printDebug();

        cout << "\nScaling with factor " << scaleFactor << " ..." << std::endl;
        ms->scale(scaleFactor);

        cout << "MedialSphere AFTER Scaling:" << std::endl;
        ms->printDebug();
        cout << endl;
    }

    void TestCases::testSphereSelection(std::vector<MedialSpherePtr>& spheres)
    {
        cout << "\n---> testSphereSelection" << endl;

        //    cout << "Eigenschaften des aktuellen Kugelvektors (von Hand):" << endl;
        //    float minRadius = SphereHelpers::findMinSphereRadius(spheres);
        //    float maxRadius = SphereHelpers::findMaxSphereRadius(spheres);
        //    cout << "minRadius " << minRadius << endl;
        //    cout << "maxRadius " << maxRadius << endl;

        //    float minObjectAngle = SphereHelpers::findMinObjectAngleDegrees(spheres);
        //    float maxObjectAngle = SphereHelpers::findMaxObjectAngleDegrees(spheres);
        //    cout << "minObjectAngle " << minObjectAngle << endl;
        //    cout << "maxObjectAngle " << maxObjectAngle << endl;

        //funktioniert nicht richtig...? --> Doch, aber verschachtelte Konstruktoren gehen nicht in dieser Version des Standards...
        SphereConstraintsSet scs = SphereHelpers::getSphereConstraintsSet(spheres);
        cout << "Eigenschaften des aktuellen Kugelvektors (mit SphereConstraintsSet scs):" << endl;
        scs.printDebug();


        //    //funktioniert
        //    SphereConstraintsSet scs2;
        //    cout << "scs2:" << endl;
        //    scs2.minRadius = minRadius;
        //    scs2.printDebug();

        //    //funktioniert
        //    SphereConstraintsSet scs3(minRadius, maxRadius, minObjectAngle, maxObjectAngle);
        //    cout << "scs3:" << endl;
        //    scs3.printDebug();

        //    //funktioniert??
        //    SphereConstraintsSet scs4(minRadius, maxRadius, minObjectAngle, maxObjectAngle, spheres.size());
        //    cout << "scs4:" << endl;
        //    scs4.printDebug();


        // ------
        //    cout << "\nSelektiere Kugeln mit >120 Grad Objektwinkel" << endl;
        //    SphereConstraintsSet scs5;
        //    scs5.minObjectAngle = 120;
        //    SphereHelpers::selectSpheresWithConstraints(spheres, scs5);
        //    SphereConstraintsSet scs6 = SphereHelpers::getSphereConstraintsSet(spheres);
        //    scs6.printDebug();

        cout << "\n scs6: " << endl;
        SphereConstraintsSet scs6 = SphereHelpers::getSphereConstraintsSet(spheres);
        scs6.printDebug();

        //    cout << "\nSelektiere Kugeln mit mind. 30% des Maximalradius" << endl;
        //    SphereConstraintsSet scs7;
        //    scs7.minRadius = 0.3*scs6.maxRadius;
        //    cout << "\n scs7: " << endl;
        //    scs7.printDebug();

        //    SphereHelpers::selectSpheresWithConstraints(spheres, scs7);
        //    SphereConstraintsSet scs8 = SphereHelpers::getSphereConstraintsSet(spheres);
        //    cout << "\n scs8: " << endl;
        //    scs8.printDebug();


        // ------
        cout << "\nSelektiere Kugeln mit >120 Grad und mid. 30% des Maximalradius" << endl;
        SphereConstraintsSet scs9;
        scs9.minObjectAngle = 120;
        scs9.minRadius = 0.3f * scs6.maxRadius;
        cout << "\n scs9: " << endl;
        scs9.printDebug();
        SphereHelpers::selectSpheresWithConstraints(spheres, scs9);

        SphereConstraintsSet scs10 = SphereHelpers::getSphereConstraintsSet(spheres);
        cout << "\n scs10: " << endl;
        scs10.printDebug();
    }

    //void TestCases::testSomeStuff()
    //{
    //    //----------- test calc stuff

    //    Eigen::Vector3f a(1,2,3);
    //    Eigen::Vector3f b(0,1,2);
    //    Eigen::Vector3f c(1,0,0);


    //    float ab = a.dot(b);
    //    printf("a dot b: %f \n", ab);

    //    float angle_bc_rad = SphereHelpers::calcAngleBetweenTwoVectorsRad(b,c);
    //    float angle_bc_deg = SphereHelpers::calcAngleBetweenTwoVectorsDeg(b,c);
    //    printf("angle_bc_rad: %f \n", angle_bc_rad);
    //    printf("angle_bc_deg: %f \n", angle_bc_deg);

    //    //----------- test MedialSphere stuff

    //    Eigen::Vector3f center(0,0,0);
    //    float sphereRadius = 1.0f;
    //    Eigen::Vector3f p1(-1,0,0);
    //    Eigen::Vector3f p2(0,-1,0);
    //    Eigen::Vector3f p3(1,0,0);
    //    Eigen::Vector3f p4(0,1,0);
    //    vector<Eigen::Vector3f> surfacePoints;
    //    surfacePoints.push_back(p1);
    //    surfacePoints.push_back(p2);
    //    surfacePoints.push_back(p3);
    //    surfacePoints.push_back(p4);

    //    MedialSphere myTestSphere(center, sphereRadius, surfacePoints);

    //    myTestSphere.printDebug();

    //    //----------- test Grid stuff

    ////    GridOfMedialSpheres goms;
    ////    vector< Eigen::Vector3f > surfacePointsNew;
    ////    vector<MedialSphere> spheres;

    //    //goms.setup(surfacePointsNew, spheres);

    //    //TODO: HIER WEITER!

    //    //--------- test nested vectors (MP 2013-06-20)
    ////    vector<int> myTestVect;
    ////    myTestVect.reserve(100);
    //////    myTestVect.push_back(1);
    //////    myTestVect.push_back(3);
    //////    myTestVect.push_back(7);

    ////    for (int i=0; i<myTestVect.capacity(); i++)
    ////        myTestVect.push_back(0);

    ////    myTestVect.at(0) = 1;
    ////    myTestVect.at(1) = 3;
    ////    myTestVect.at(2) = 7;

    ////    cout << "myTestVect.capacity: " << myTestVect.capacity() << endl;
    ////    cout << "myTestVect.size: " << myTestVect.size() << endl;
    ////    myTestVect.at(9) = 12;
    ////    cout << "myTestVect.size: " << myTestVect.size() << endl;

    ////    cout << "myTestVect contents: " << endl;
    ////    for (int i=0; i<myTestVect.size(); i++)
    ////        cout << myTestVect.at(i) << " ";
    ////    cout << endl;

    //    //---------- now 2D

    ////    vector< vector<int> > myTestVect2D;
    ////    myTestVect2D.reserve(5);
    ////    cout << "myTestVect2D.capacity: " << myTestVect2D.capacity() << endl;
    ////    cout << "myTestVect2D.size: " << myTestVect2D.size() << endl;
    //////    cout << "myTestVect2D.at(0).capacity: " << myTestVect2D.at(0).capacity() << endl;
    //////    cout << "myTestVect2D.at(0).size: " << myTestVect2D.at(0).size() << endl;


    ////    vector<int> vTest1;
    ////    vector<int> vTest2;
    ////    vTest1.reserve(2);
    ////    vTest2.reserve(2);
    ////    vTest1.push_back(1);
    ////    vTest2.push_back(2);
    ////    vTest1.push_back(1);
    ////    vTest2.push_back(2);


    ////    for (int i=0; i<myTestVect2D.capacity(); i++)
    ////    {


    ////       myTestVect2D.push_back(vTest1);
    ////       //myTestVect2D.push_back(vTest2);
    ////       //myTestVect2D.at(i).push_back(i);
    ////    }

    ////    cout << "myTestVect2D.capacity: " << myTestVect2D.capacity() << endl;
    ////    cout << "myTestVect2D.size: " << myTestVect2D.size() << endl;
    ////    myTestVect2D.at(3).push_back(900);
    ////    myTestVect2D.at(2).at(1) = 400;
    ////    myTestVect2D[1][0] = 200;
    ////    cout << "myTestVect2D.size: " << myTestVect2D.size() << endl;

    ////    cout << "myTestVect2D contents: " << endl;
    ////    for (int i=0; i<myTestVect2D.size(); i++)
    ////    {
    ////        for (int j=0; j<myTestVect2D.at(i).size(); j++)
    ////            cout << myTestVect2D.at(i).at(j) << " ";
    ////        cout << endl;
    ////    }
    ////    cout << endl;

    ////    for (int i=0; i<myTestVect2D.size(); i++)
    ////    {
    ////        cout << "myTestVect2D.at(i).capacity: " << myTestVect2D.at(i).capacity() << endl;
    ////        cout << "myTestVect2D.at(i).size: " << myTestVect2D.at(i).size() << endl;
    ////    }

    ////    // ---------------------- now 3D

    ////    int xdim = 2;
    ////    int ydim = 3;
    ////    int zdim = 4;

    ////    vector<int> v1D;
    ////    vector< vector<int> > v2D;
    ////    vector< vector< vector<int> > > v3D;

    ////    v1D.reserve(xdim);
    ////    v1D.push_back(1);
    ////    v1D.push_back(1);

    ////    v2D.reserve(ydim);
    ////    for (int j=0; j<v2D.capacity(); j++)
    ////        v2D.push_back(v1D);

    ////    v3D.reserve(zdim);
    ////    for (int k=0; k<v3D.capacity(); k++)
    ////        v3D.push_back(v2D);

    ////    v3D.at(3).at(0).push_back(900);
    ////    v3D.at(2).at(1).at(0) = 400;
    ////    v3D[1][0][0] = 200;

    ////    for (int x=0; x<v3D.size(); x++)
    ////    {
    ////        for (int y=0; y<v3D.at(x).size(); y++)
    ////        {
    ////            for (int z=0; z<v3D.at(x).at(y).size(); z++)
    ////                cout << v3D.at(x).at(y).at(z) << " ";
    ////            cout << endl;
    ////        }
    ////        cout << endl;
    ////    }

    //    // ---------------------- now 4D

    ////    int xdim = 2;
    ////    int ydim = 3;
    ////    int zdim = 4;
    ////    int listLength = 10;

    ////    vector<int> v1D;
    ////    vector< vector<int> > v2D;
    ////    vector< vector< vector<int> > > v3D;
    ////    vector< vector< vector< vector<int> > > > v4D;

    ////    v1D.reserve(listLength);
    ////    //v1D.push_back(1);
    ////    //v1D.push_back(1);

    ////    //v2D.reserve(xdim);
    ////    v2D.reserve(zdim);
    ////    for (int j=0; j<v2D.capacity(); j++)
    ////        v2D.push_back(v1D);

    ////    v3D.reserve(ydim);
    ////    for (int k=0; k<v3D.capacity(); k++)
    ////        v3D.push_back(v2D);

    ////    //v4D.reserve(zdim);
    ////    v4D.reserve(xdim);
    ////    for (int l=0; l<v4D.capacity(); l++)
    ////        v4D.push_back(v3D);

    ////    //v3D.at(3).at(0).push_back(900);
    ////    v4D.at(0).at(1).at(2).push_back(400);
    ////    v4D[1][0][0].push_back(200);
    ////    v4D[1][0][0].push_back(300);

    ////    for (int x=0; x<v4D.size(); x++)
    ////    {
    ////        for (int y=0; y<v4D.at(x).size(); y++)
    ////        {
    ////            for (int z=0; z<v4D.at(x).at(y).size(); z++)
    ////            {
    ////                cout << "capacity v4D.at(x).at(y).at(z): ["
    ////                     << x << " " << y << " " << z << " " << "] "
    ////                     << v4D.at(x).at(y).at(z).capacity()
    ////                     << " size v4D.at(x).at(y).at(z): " << v4D.at(x).at(y).at(z).size() << endl;
    ////                if (v4D.at(x).at(y).at(z).size() > 0)
    ////                {
    ////                    cout << "Content: ";
    ////                    for (int q=0; q<v4D.at(x).at(y).at(z).size(); q++)
    ////                        cout <<v4D.at(x).at(y).at(z).at(q) << " ";
    ////                    cout << endl;
    ////                }

    ////            }

    ////            cout << endl;
    ////        }
    ////        cout << endl;
    ////    }


    //    //------- test build grid

    //    Eigen::Vector3i nCells(2,3,4);
    ////    nCells.push_back(2);
    ////    nCells.push_back(3);
    ////    nCells.push_back(4);


    ////    //von oben...
    ////    Eigen::Vector3f center2(0,0,0);
    ////    float sphereRadius2 = 1.0f;

    //    std::vector<MedialSphere> spheres;

    //    //Kugel 1
    //    center << 0, 1, 0;
    //    sphereRadius = 1.0f;
    //    p1 << 0, 2, 0;
    //    p2 << 0, 1, 1;
    //    surfacePoints.clear();
    //    surfacePoints.push_back(p1);
    //    surfacePoints.push_back(p2);
    //    MedialSphere sphere1(center, sphereRadius, surfacePoints);
    //    spheres.push_back(sphere1);

    //    //Kugel 2
    //    center << 0, 1, 1;
    //    sphereRadius = 2.0f;
    //    p1 << 0, 3, 1;
    //    p2 << 0, -1, 1;
    //    surfacePoints.clear();
    //    surfacePoints.push_back(p1);
    //    surfacePoints.push_back(p2);
    //    MedialSphere sphere2(center, sphereRadius, surfacePoints);
    //    spheres.push_back(sphere2);

    //    //Kugel 3
    //    center << 1, 0, 0;
    //    sphereRadius = 1.0f;
    //    p1 << 1, 0, 1;
    //    p2 << 1, 1, 0;
    //    p3 << 2, 0, 0;
    //    surfacePoints.clear();
    //    surfacePoints.push_back(p1);
    //    surfacePoints.push_back(p2);
    //    surfacePoints.push_back(p3);
    //    MedialSphere sphere3(center, sphereRadius, surfacePoints);
    //    spheres.push_back(sphere3);

    //    for (int i=0; i<spheres.size(); i++)
    //        spheres.at(i).printDebug();



    ////    Eigen::Vector3f p1_2(-1,0,0);
    ////    Eigen::Vector3f p2_2(0,-1,0);
    ////    Eigen::Vector3f p3_2(1,0,0);
    ////    Eigen::Vector3f p4_2(0,1,0);

    ////    vector<Eigen::Vector3f> surfacePoints2;
    ////    surfacePoints2.push_back(p1_2);
    ////    surfacePoints2.push_back(p2_2);
    ////    surfacePoints2.push_back(p3_2);
    ////    surfacePoints2.push_back(p4_2);



    //    GridOfMedialSpheres goms;
    //    goms.allocateGrid(nCells);
    //    for (int i=0; i<spheres.size(); i++)
    //        goms.insertSphere(spheres.at(i));

    //    //------- test cube index computation

    ////    Eigen::Vector3i seedIndex(0,1,0);
    ////    Eigen::Vector3i maxCoords(2,2,3);
    ////    int cubeRadiusToSearch = 1;
    ////    std::vector<Eigen::Vector3i> cubeIndices;

    ////    cubeIndices = goms.computeCubeIndices(seedIndex, maxCoords, cubeRadiusToSearch);

    ////    std::cout << "\nTest computeCubeIndices()" << std::endl;
    ////    std::cout << "seedIndex: " << StrOutHelpers::toString(seedIndex)
    ////              << "maxCoords: " << StrOutHelpers::toString(maxCoords)
    ////              << "cubeRadiusToSearch: " << cubeRadiusToSearch
    ////              << std::endl;
    ////    std::cout << StrOutHelpers::toString(cubeIndices);

    //}

    void TestCases::testSetupGrid(std::vector<MedialSpherePtr>& spheres, std::vector<Eigen::Vector3f> surfacePoints)
    {
        cout << "\n-----\nTestCases::testSetupGrid() called." << endl;

        GridOfMedialSpheres goms;
        int gridConstant = 4;
        goms.setup(surfacePoints, spheres, gridConstant);   //hoho...!

        goms.printDebugGridCells();
        // HIER WEITER!!

        //----------
        cout << "\n Now get some spheres from the grid." << endl;
        //Tested with the salt box test object.

        Eigen::Vector3i index3d(7, 5, 10);
        std::vector<Eigen::Vector3i> gridIndices;
        gridIndices.push_back(index3d);
        cout << "Get spheres from cell " << StrOutHelpers::toString(index3d) << endl;
        std::vector<MedialSpherePtr> spheresInOneCell = goms.getSpheresFromGrid(gridIndices);
        cout << "I fetched " << spheresInOneCell.size() << " spheres." << endl;
        gridIndices.clear();
        spheresInOneCell.clear();

        index3d << 7, 4, 2;
        gridIndices.push_back(index3d);
        cout << "Get spheres from cell " << StrOutHelpers::toString(index3d) << endl;
        spheresInOneCell = goms.getSpheresFromGrid(gridIndices);
        cout << "I fetched " << spheresInOneCell.size() << " spheres." << endl;
        gridIndices.clear();
        spheresInOneCell.clear();

        index3d << 6, 2, 0;
        gridIndices.push_back(index3d);
        cout << "Get spheres from cell " << StrOutHelpers::toString(index3d) << endl;
        spheresInOneCell = goms.getSpheresFromGrid(gridIndices);
        cout << "I fetched " << spheresInOneCell.size() << " spheres." << endl;
        gridIndices.clear();
        spheresInOneCell.clear();

        index3d << 0, 0, 0;
        gridIndices.push_back(index3d);
        cout << "Get spheres from cell " << StrOutHelpers::toString(index3d) << endl;
        spheresInOneCell = goms.getSpheresFromGrid(gridIndices);
        cout << "I fetched " << spheresInOneCell.size() << " spheres." << endl;
        gridIndices.clear();
        spheresInOneCell.clear();

        //-----
        cout << "\nTest isGridIndexValid()" << endl;
        gridIndices.clear();

        index3d << 0, 0, 0;
        gridIndices.push_back(index3d);
        index3d << -1, 0, 0;
        gridIndices.push_back(index3d);
        index3d << 0, -1, 0;
        gridIndices.push_back(index3d);
        index3d << 0, 0, -1;
        gridIndices.push_back(index3d);
        index3d << 8, 6, 17;
        gridIndices.push_back(index3d);
        index3d << 8, -1, 17;
        gridIndices.push_back(index3d);
        index3d << 8, 6, -1;
        gridIndices.push_back(index3d);
        index3d << -19, 6, 17;
        gridIndices.push_back(index3d);
        index3d << 8, 6, 17;
        gridIndices.push_back(index3d);
        index3d << 7, 5, 16;
        gridIndices.push_back(index3d);
        index3d << 8, 5, 16;
        gridIndices.push_back(index3d);
        index3d << 3, 6, 9;
        gridIndices.push_back(index3d);
        index3d << 8, 6, 17;
        gridIndices.push_back(index3d);
        index3d << 8, 6, 18;
        gridIndices.push_back(index3d);
        index3d << 8, 7, 17;
        gridIndices.push_back(index3d);
        index3d << 9, 6, 17;
        gridIndices.push_back(index3d);
        index3d << -1, 6, 18;
        gridIndices.push_back(index3d);
        index3d << 8, 7, -1;
        gridIndices.push_back(index3d);
        index3d << 9, -1, 17;
        gridIndices.push_back(index3d);

        //bool isValid;
        //    for (int i=0; i<gridIndices.size(); i++)
        //    {
        //        isValid = goms.isGridIndexValid(gridIndices.at(i));
        //        cout << "gridIndex " << StrOutHelpers::toString(gridIndices.at(i))
        //             << " valid? "   << isValid << endl;
        //    }

        //---- Test computeCubeIndices() ----
        cout << "\n\nNow testing computeCubeIndices()" << endl;

        int maxCubeRadiusToSearch = 2;
        Eigen::Vector3i maxCubeIndex(7, 5, 10); //entspricht numCells(i)-1

        Eigen::Vector3i seedIndex(2, 2, 2); //entspricht numCells(i)-1
        //    testComputeCubeIndices(goms, seedIndex, maxCubeIndex, maxCubeRadiusToSearch);

        //    seedIndex << 0,0,0;
        //    testComputeCubeIndices(goms, seedIndex, maxCubeIndex, maxCubeRadiusToSearch);

        //    seedIndex << 7,5,10;
        //    testComputeCubeIndices(goms, seedIndex, maxCubeIndex, maxCubeRadiusToSearch);


        //---- Test getSpheresFromGrid()----
        //cout << "\n\n Now testing getSpheresFromGrid()" << endl;

        int cubeRadius = 1;
        std::vector<Eigen::Vector3i> cubeIndices;
        std::vector<MedialSpherePtr> returnSpheres;

        seedIndex << 2, 2, 2;
        cubeIndices = testGetComputeCubeIndices(goms, seedIndex,
                                                maxCubeIndex, cubeRadius);
        returnSpheres = goms.getSpheresFromGrid(cubeIndices);
        cout << "I got " << returnSpheres.size() << " spheres from the grid. " << endl;
        cubeIndices.clear();
        returnSpheres.clear();

        seedIndex << 0, 0, 0;
        cubeIndices = testGetComputeCubeIndices(goms, seedIndex,
                                                maxCubeIndex, cubeRadius);
        returnSpheres = goms.getSpheresFromGrid(cubeIndices);
        cout << "I got " << returnSpheres.size() << " spheres from the grid. " << endl;
        cubeIndices.clear();
        returnSpheres.clear();


        seedIndex << 7, 5, 10;
        cubeIndices = testGetComputeCubeIndices(goms, seedIndex,
                                                maxCubeIndex, cubeRadius);
        returnSpheres = goms.getSpheresFromGrid(cubeIndices);
        cout << "I got " << returnSpheres.size() << " spheres from the grid. " << endl;
        cubeIndices.clear();
        returnSpheres.clear();



    }

    void TestCases::testComputeCubeIndices(GridOfMedialSpheres& goms, Eigen::Vector3i seedIndex, Eigen::Vector3i maxCubeIndex, int maxCubeRadiusToSearch)
    {
        std::vector<Eigen::Vector3i> cubeIndices;

        for (int cubeRadius = 0; cubeRadius <= maxCubeRadiusToSearch; cubeRadius++)
        {
            cubeIndices = goms.computeCubeIndices(seedIndex, maxCubeIndex, cubeRadius);
            cout << "\nmaxCubeIndex " << StrOutHelpers::toString(maxCubeIndex) << endl;
            cout << "seedIndex " << StrOutHelpers::toString(seedIndex) << endl;
            cout << "cubeRadiusToSearch " << cubeRadius << endl;
            cout << "I got the following cubeIndices back, " << cubeIndices.size()
                 << " total: " << StrOutHelpers::toString(cubeIndices, false) << endl;
        }
    }

    std::vector<Eigen::Vector3i> TestCases::testGetComputeCubeIndices(GridOfMedialSpheres& goms, Eigen::Vector3i seedIndex, Eigen::Vector3i maxCubeIndex, int cubeRadius)
    {
        std::vector<Eigen::Vector3i> cubeIndices;

        cubeIndices = goms.computeCubeIndices(seedIndex, maxCubeIndex, cubeRadius);
        cout << "\nmaxCubeIndex " << StrOutHelpers::toString(maxCubeIndex) << endl;
        cout << "seedIndex " << StrOutHelpers::toString(seedIndex) << endl;
        cout << "cubeRadiusToSearch " << cubeRadius << endl;
        cout << "I got the following cubeIndices back, " << cubeIndices.size()
             << " total: " << StrOutHelpers::toString(cubeIndices, false) << endl;

        return cubeIndices;
    }

    void TestCases::testEigenMathFunctions()
    {
        Eigen::Matrix3f m;
        m << 1, 2, 3,
        4, 5, 6,
        7, 8, 9;
        std::cout << "matrix m: " << std::endl;
        std::cout << m << std::endl;

        float sumOfMatrix = m.sum();
        std::cout << "sum: " << sumOfMatrix << std::endl;

        float meanOfMatrix = m.mean();
        std::cout << "mean: " << meanOfMatrix << std::endl;

        Eigen::Vector3f colSum = m.colwise().sum();
        //std::cout << "colSum: " << StrOutHelpers::toString(colSum) << std::endl;
        std::cout << "colSum: " << colSum << std::endl;

        Eigen::Vector3f rowSum = m.rowwise().sum();
        //std::cout << "rowSum: " << StrOutHelpers::toString(rowSum) << std::endl;
        std::cout << "rowSum: " << rowSum << std::endl;

        Eigen::Vector3f colMean = m.colwise().mean();
        //std::cout << "colMean: " << StrOutHelpers::toString(colMean) << std::endl;
        std::cout << "colMean: " << colMean << std::endl;

        Eigen::Vector3f rowMean = m.colwise().mean();
        //std::cout << "rowMean: " << StrOutHelpers::toString(rowMean) << std::endl;
        std::cout << "rowMean: " << rowMean << std::endl;

        //Eigen::Matrix<float, 3, Eigen::Dynamic> m2;
        //Eigen::MatrixXf m2(3,2);
        //Eigen::MatrixXf m2(3,5);
        Eigen::MatrixXf m2;
        m2.resize(3, 2);

        m2 << rowSum, rowSum;
        std::cout << "m2: " << std::endl;
        std::cout << m2 << std::endl;

        m2.resize(3, 4);
        m2.setZero();

        std::cout << "m2: " << std::endl;
        std::cout << m2 << std::endl;

        m2.block<3, 1>(0, 2) = rowMean;
        m2.block<3, 1>(0, 3) = rowSum;

        //    m2 << rowMean;
        //    m2 << rowMean;


        //    m2 << m;
        std::cout << "m2: " << std::endl;
        std::cout << m2 << std::endl;

        // ----
        std::vector<Eigen::Vector3f> myTestVect;
        myTestVect.push_back(rowSum);
        myTestVect.push_back(rowMean);
        myTestVect.push_back(rowSum);
        myTestVect.push_back(rowMean);
        myTestVect.push_back(rowSum);
        myTestVect.push_back(colSum);
        myTestVect.push_back(colMean);


        Eigen::MatrixXf m3 = SphereHelpers::toMatrix_3xN(myTestVect);
        std::cout << "m3: " << std::endl;
        std::cout << m3 << std::endl;

        Eigen::Vector3f cog = m3.rowwise().mean();
        std::cout << "mean of points in m3: " << cog << std::endl;

        // --- Eigenvalues and eigenvectors via SVD:
        //Eigen::MatrixXf m4 = Eigen::MatrixXf::Random(3,2);
        Eigen::MatrixXf m4;
        m4.resize(3, 6);
        m4 << 10, 2, 3, 4, 5, 14,
        10, 2, 3, 4, 5, 14,
        0, 0, 0, 0, 0, 0;

        std::cout << "Here is the matrix m4: " << std::endl << m4 << std::endl;
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(m4, Eigen::ComputeThinU | Eigen::ComputeThinV);
        std::cout << "Its singular values are: " << std::endl << svd.singularValues() << std::endl;
        std::cout << "Its left singular vectors are the columns of the thin U matrix: "
                  << std::endl << svd.matrixU() << std::endl;
        std::cout << "Its right singular vectors are the columns of the thin V matrix: "
                  << std::endl << svd.matrixV() << std::endl;
        Eigen::Vector3f rhs(1, 0, 0);
        std::cout << "Now consider this rhs vector: " << std::endl << rhs << std::endl;
        std::cout << "A least-squares solution of m*x = rhs is: " << std::endl << svd.solve(rhs)
                  << std::endl;


        //HIER WEITER!



    }

}
