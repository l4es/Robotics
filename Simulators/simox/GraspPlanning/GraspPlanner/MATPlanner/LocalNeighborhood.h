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
#ifndef LOCALNEIGHBORHOOD_H
#define LOCALNEIGHBORHOOD_H

#include "../../GraspStudio.h"
#include <vector>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include "SphereHelpers.h"
#include "StrOutHelpers.h"

namespace GraspStudio
{

    class LocalNeighborhood;
    typedef boost::shared_ptr<LocalNeighborhood> LocalNeighborhoodPtr;

    class GRASPSTUDIO_IMPORT_EXPORT LocalNeighborhood
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LocalNeighborhood();
        LocalNeighborhood(MedialSpherePtr seedSphere, std::vector<MedialSpherePtr>& spheresInNeighborhood, float neighborhoodRadius);

        MedialSpherePtr centerSphere;
        Eigen::Vector3f center;
        std::vector<MedialSpherePtr> spheres;
        float radius;
        size_t numberOfSpheres;
        bool isEmpty;
        bool hasBeenAnalyzed;

        //evaluation of local symmetry properties
        Eigen::Vector3f eigenvector1;
        Eigen::Vector3f eigenvector2;
        float eigenvalue1;
        float eigenvalue2;
        float ratioOfEigenvalues;

        Eigen::Vector3f centerOfGravity;

        void determineEigenvectorsAndEigenvalues_viaSVD();
        //void determineEigenvectorsAndEigenvalues_viaCovMatrix();
        void computeCenterOfGravity();
        void analyze();
        void printDebug();
        bool isValid();

    };
}
#endif // LOCALNEIGHBORHOOD_H
