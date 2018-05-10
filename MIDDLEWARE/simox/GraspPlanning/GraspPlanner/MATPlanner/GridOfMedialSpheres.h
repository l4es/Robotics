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
#ifndef GRIDOFMEDIALSPHERES_H
#define GRIDOFMEDIALSPHERES_H


#include "../../GraspStudio.h"
#include "MedialSphere.h"
#include "GridParameters.h"
#include "SphereHelpers.h"

#include <vector>
#include <Eigen/Geometry>

namespace GraspStudio
{

    class GridOfMedialSpheres;
    typedef boost::shared_ptr<GridOfMedialSpheres> GridOfMedialSpheresPtr;

    class GRASPSTUDIO_IMPORT_EXPORT GridOfMedialSpheres
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GridOfMedialSpheres(bool verbose = false);
        ~GridOfMedialSpheres();

        void setup(std::vector< Eigen::Vector3f > surfacePoints, std::vector<MedialSpherePtr> spheres, int gridConstant = 4);

        void reset();

        void allocateGrid(Eigen::Vector3i numCells);
        std::vector<Eigen::Vector3i> computeCubeIndices(Eigen::Vector3i seedIndex, Eigen::Vector3i maxCoords, int cubeRadiusToSearch);


        //methods for access
        void insertSphere(MedialSpherePtr s);

        std::vector<MedialSpherePtr> getSpheresFromGrid(std::vector<Eigen::Vector3i> gridIndices);
        std::vector<MedialSpherePtr> getSpheresInNeighborhood(MedialSpherePtr s, float searchRadius);

        Eigen::Vector3i calcGridIndex(Eigen::Vector3f sphereCenter);
        bool isGridIndexValid(Eigen::Vector3i gridIndex);

        //for debugging
        void printDebugGridCells();

    protected:

        //methods for setting up the grid
        void computeGrid();

        GridParameters determineGridParameters(std::vector<Eigen::Vector3f> surfacePoints, std::vector<MedialSpherePtr> spheres, int gridConstant);
        void findMinMax(std::vector<Eigen::Vector3f> surfacePoints, Eigen::Vector3f& minPoint, Eigen::Vector3f& maxPoint);
        void determineCellNumbers(std::vector<Eigen::Vector3f> surfacePoints, Eigen::Vector3f minPoint, Eigen::Vector3f maxPoint, int gridConstant, Eigen::Vector3i& numCells, float& cellWidth);
        void buildGrid(std::vector<MedialSpherePtr> spheres, GridParameters gridParams);

        bool verbose;

        GridParameters gridParameters;

        //the actual grid of spheres
        std::vector< std::vector< std::vector< std::vector< MedialSpherePtr > > > > grid;

        int numberOfSpheres;
        float minRadius;
        float maxRadius;

    };
}
#endif // GRIDOFMEDIALSPHERES_H
