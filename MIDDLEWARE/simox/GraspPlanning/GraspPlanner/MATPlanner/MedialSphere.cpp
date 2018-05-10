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
#include "MedialSphere.h"
#include "SphereHelpers.h"

using namespace std;
namespace GraspStudio
{

    MedialSphere::MedialSphere()
    {
    }

    MedialSphere::MedialSphere(Eigen::Vector3f center, float radius, vector<Eigen::Vector3f> surfacePointCoordinates)
    {
        this->center = center;
        this->radius = radius;
        this->surfacePointCoordinates = surfacePointCoordinates;

        this->computeVectorsToSurfacePointsAndObjectAngle();

    }

    MedialSphere::~MedialSphere()
    {
    }

    MedialSpherePtr MedialSphere::clone()
    {
        MedialSpherePtr cloneSphere = MedialSpherePtr(new MedialSphere);
        cloneSphere->center = this->center;
        cloneSphere->radius = this->radius;
        cloneSphere->surfacePointCoordinates = this->surfacePointCoordinates;
        cloneSphere->vectorsToSurfacePoints = this->vectorsToSurfacePoints;
        cloneSphere->objectAngle = this->objectAngle;

        return cloneSphere;
    }


    void MedialSphere::computeVectorsToSurfacePointsAndObjectAngle()
    {
        Eigen::Vector3f v;

        vectorsToSurfacePoints.clear();

        for (size_t i = 0; i < surfacePointCoordinates.size(); i++)
        {
            v = surfacePointCoordinates.at(i) - center;
            this->vectorsToSurfacePoints.push_back(v);
        }

        float currentAngleDeg = -1;
        float maxAngleDeg = -1;

        for (size_t j = 0; j < vectorsToSurfacePoints.size(); j++)
            for (size_t k = j + 1; k < vectorsToSurfacePoints.size(); k++)
            {
                currentAngleDeg = SphereHelpers::calcAngleBetweenTwoVectorsDeg(vectorsToSurfacePoints.at(j), vectorsToSurfacePoints.at(k));

                if (currentAngleDeg > maxAngleDeg)
                {
                    maxAngleDeg = currentAngleDeg;
                }
            }

        this->objectAngle = maxAngleDeg;

    }

    void MedialSphere::printDebug()
    {
        std::cout << "=== MedialSphere ===" << std::endl;
        std::cout << "center: " << center(0) << " " << center(1) << " " << center(2) << std::endl;
        std::cout << "radius: " << radius << std::endl;
        std::cout << "surfacePointCoordinates: " << StrOutHelpers::toString(surfacePointCoordinates) << std::endl;
        std::cout << "vectorsToSurfacePoints: " << StrOutHelpers::toString(vectorsToSurfacePoints) << std::endl;
        std::cout << "objectAngle: " << objectAngle << std::endl;
    }

    void MedialSphere::scale(float scaleFactor)
    {
        radius = scaleFactor * radius;
        center = scaleFactor * center;

        //objectAngle stays the same! No scaling!

        for (size_t i = 0; i < surfacePointCoordinates.size(); i++)
        {
            surfacePointCoordinates.at(i) = scaleFactor * surfacePointCoordinates.at(i);
        }

        computeVectorsToSurfacePointsAndObjectAngle();

    }


}
