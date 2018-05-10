#ifndef TESTCASES_H
#define TESTCASES_H

#include "../../GraspStudio.h"
#include "SphereHelpers.h"
#include "GridOfMedialSpheres.h"
namespace GraspStudio
{

    class GRASPSTUDIO_IMPORT_EXPORT TestCases
    {
    public:
        TestCases();

        //static void testSomeStuff();

        static void testScaleMedialSphere();
        static void testScaleMedialSphere(MedialSpherePtr ms, float scaleFactor);

        static void testSphereSelection(std::vector<MedialSpherePtr>& spheres);

        static void testSetupGrid(std::vector<MedialSpherePtr>& spheres, std::vector<Eigen::Vector3f> surfacePoints);

        static void testComputeCubeIndices(GridOfMedialSpheres& goms, Eigen::Vector3i seedIndex, Eigen::Vector3i maxCubeIndex, int maxCubeRadiusToSearch);
        static std::vector<Eigen::Vector3i> testGetComputeCubeIndices(GridOfMedialSpheres& goms, Eigen::Vector3i seedIndex, Eigen::Vector3i maxCubeIndex, int maxCubeRadiusToSearch);

        static void testEigenMathFunctions();


    };
}
#endif // TESTCASES_H
