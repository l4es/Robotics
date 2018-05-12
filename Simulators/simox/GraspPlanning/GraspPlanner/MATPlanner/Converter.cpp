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
#include "Converter.h"

using namespace std;
using namespace Eigen;

namespace GraspStudio
{

    Converter::Converter()
    {
    }


    MedialSpherePtr Converter::convertPowerCrustPolarBallToMedialSphere(GraspStudio::PowerCrust::PolarBall& pB)
    {
        Vector3f center;
        float radius;
        Vector3f tempSurfacePoint;
        vector<Vector3f> surfacePoints;

        center << (float)pB.poleCenter[0],
               (float)pB.poleCenter[1],
               (float)pB.poleCenter[2];
        radius = (float)pB.poleRadius;

        tempSurfacePoint << (float)pB.surfacePoint1[0],
                         (float)pB.surfacePoint1[1],
                         (float)pB.surfacePoint1[2];
        surfacePoints.push_back(tempSurfacePoint);
        tempSurfacePoint << (float)pB.surfacePoint2[0],
                         (float)pB.surfacePoint2[1],
                         (float)pB.surfacePoint2[2];
        surfacePoints.push_back(tempSurfacePoint);
        tempSurfacePoint << (float)pB.surfacePoint3[0],
                         (float)pB.surfacePoint3[1],
                         (float)pB.surfacePoint3[2];
        surfacePoints.push_back(tempSurfacePoint);
        tempSurfacePoint << (float)pB.surfacePoint4[0],
                         (float)pB.surfacePoint4[1],
                         (float)pB.surfacePoint4[2];
        surfacePoints.push_back(tempSurfacePoint);

        MedialSpherePtr ms = MedialSpherePtr(new MedialSphere(center, radius, surfacePoints));

        return ms;
    }

    std::vector<MedialSpherePtr> Converter::convertPowerCrustPolarBallsToMedialSpheres(std::vector<GraspStudio::PowerCrust::PolarBall>& polarBalls)
    {
        std::vector<MedialSpherePtr> allSpheres;

        for (size_t i = 0; i < polarBalls.size(); i++)
        {
            MedialSpherePtr ms = convertPowerCrustPolarBallToMedialSphere(polarBalls.at(i));
            allSpheres.push_back(ms);
        }

        return allSpheres;
    }
}
