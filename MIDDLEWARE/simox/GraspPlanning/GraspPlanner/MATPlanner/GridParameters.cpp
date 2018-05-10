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
#include "GridParameters.h"

using namespace std;
using namespace Eigen;

namespace GraspStudio
{

    GridParameters::GridParameters()
    {
    }

    GridParameters::GridParameters(Vector3i numCells, Vector3f minPoint, Vector3f maxPoint, float cellWidth)
    {
        this->numCells = numCells;
        this->minPoint = minPoint;
        this->maxPoint = maxPoint;
        this->cellWidth = cellWidth;

        maxGridIndex(0) = numCells(0) - 1;
        maxGridIndex(1) = numCells(1) - 1;
        maxGridIndex(2) = numCells(2) - 1;
    }

}
