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
* @author     Markus Przybylski
* @copyright  2013 H2T,KIT
*             GNU Lesser General Public License
*
*/
#ifndef SPHEREHELPERS_H
#define SPHEREHELPERS_H

#include "../../GraspStudio.h"
#include "MedialSphere.h"

namespace GraspStudio
{

    class GRASPSTUDIO_IMPORT_EXPORT SphereConstraintsSet
    {
    public:
        SphereConstraintsSet();
        SphereConstraintsSet(float r_min, float r_max, float alpha_min, float alpha_max, int numSpheres = -1);

        void printDebug();

        float minRadius;
        float maxRadius;
        float minObjectAngle;
        float maxObjectAngle;

        int numberOfSpheres;
    };

    class GRASPSTUDIO_IMPORT_EXPORT SphereHelpers
    {
    public:
        SphereHelpers();

        static float findMinSphereRadius(std::vector<MedialSpherePtr>& spheres);
        static float findMaxSphereRadius(std::vector<MedialSpherePtr>& spheres);

        static float findMinObjectAngleDegrees(std::vector<MedialSpherePtr>& spheres);
        static float findMaxObjectAngleDegrees(std::vector<MedialSpherePtr>& spheres);

        static void scaleMedialSpheres(std::vector<MedialSpherePtr>& spheres, float scaleFactor);

        static void selectSpheresWithMinRadius(std::vector<MedialSpherePtr>& spheres, float minRadius);
        static void selectSpheresWithMaxRadius(std::vector<MedialSpherePtr>& spheres, float maxRadius);

        static void selectSpheresWithMinObjectAngle(std::vector<MedialSpherePtr>& spheres, float minObjectAngle);
        static void selectSpheresWithMaxObjectAngle(std::vector<MedialSpherePtr>& spheres, float maxObjectAngle);

        static void selectSpheresWithConstraints(std::vector<MedialSpherePtr>& spheres, SphereConstraintsSet constraints);

        static SphereConstraintsSet getSphereConstraintsSet(std::vector<MedialSpherePtr>& spheres);

        static void filterSpheres(std::vector<MedialSpherePtr>& spheres, float minObjAngle, float minRadiusRelative, bool verbose = false);

        static std::vector<MedialSpherePtr> getSpheresInsideSearchRadius(std::vector<MedialSpherePtr>& spheresToBeFiltered, MedialSpherePtr querySphere, float searchRadius);

        static std::vector<Eigen::Vector3f> getSphereCenters(std::vector<MedialSpherePtr>& spheres);

        //calc angle in radians
        static float calcAngleBetweenTwoVectorsRad(Eigen::Vector3f v1, Eigen::Vector3f v2);
        //calc angle in degrees
        static float calcAngleBetweenTwoVectorsDeg(Eigen::Vector3f v1, Eigen::Vector3f v2);

        static Eigen::MatrixXf toMatrix_3xN(std::vector<Eigen::Vector3f> points);

    };
}

#endif // SPHEREHELPERS_H
