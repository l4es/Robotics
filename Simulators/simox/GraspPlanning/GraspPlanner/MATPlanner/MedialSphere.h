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
#ifndef MEDIALSPHERE_H
#define MEDIALSPHERE_H

#include "../../GraspStudio.h"
#include <vector>
#include <Eigen/Geometry>

#include "StrOutHelpers.h"

namespace GraspStudio
{

    class MedialSphere;
    typedef boost::shared_ptr<MedialSphere> MedialSpherePtr;

    class GRASPSTUDIO_IMPORT_EXPORT MedialSphere
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        MedialSphere();
        MedialSphere(Eigen::Vector3f center, float radius, std::vector<Eigen::Vector3f> surfacePointCoordinates);

        ~MedialSphere();

        MedialSpherePtr clone();
        void printDebug();
        void scale(float scaleFactor);

        //data members
        Eigen::Vector3f center;
        float radius;

        std::vector<Eigen::Vector3f> surfacePointCoordinates;
        std::vector<Eigen::Vector3f> vectorsToSurfacePoints;

        float objectAngle;  //The biggest angle included between two vectors from the sphere's center to two different surface points.

    protected:
        void computeVectorsToSurfacePointsAndObjectAngle();

    };
}
#endif // MEDIALSPHERE_H
