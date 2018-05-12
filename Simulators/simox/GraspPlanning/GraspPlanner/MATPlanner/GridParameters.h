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
#ifndef GRIDPARAMETERS_H
#define GRIDPARAMETERS_H

#include <vector>
#include <Eigen/Geometry>
namespace GraspStudio
{

    class GridParameters
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GridParameters();
        GridParameters(Eigen::Vector3i numCells, Eigen::Vector3f minPoint, Eigen::Vector3f maxPoint, float cellWidth);

        Eigen::Vector3i numCells;
        Eigen::Vector3i maxGridIndex;
        Eigen::Vector3f minPoint;
        Eigen::Vector3f maxPoint;
        float cellWidth;

    };
}
#endif // GRIDPARAMETERS_H
