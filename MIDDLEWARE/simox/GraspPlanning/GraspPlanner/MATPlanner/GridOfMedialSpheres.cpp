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
#include "GridOfMedialSpheres.h"
#include <cmath>
#include <iostream>


using namespace std;
using namespace Eigen;

namespace GraspStudio
{

    GridOfMedialSpheres::GridOfMedialSpheres(bool verbose)
    {
        this->verbose = verbose;
    }

    GridOfMedialSpheres::~GridOfMedialSpheres()
    {
    }

    void GridOfMedialSpheres::reset()
    {
        numberOfSpheres = 0;
        minRadius = -1;
        maxRadius = -1;

        grid.clear();
    }


    void GridOfMedialSpheres::setup(vector< Vector3f > surfacePoints, vector<MedialSpherePtr> spheres, int gridConstant)
    {
        if (verbose)
        {
            VR_INFO << "GridOfMedialSpheres::setup() called." << endl;
        }

        GridParameters gridParams = determineGridParameters(surfacePoints, spheres, gridConstant);
        this->gridParameters = gridParams;

        //build the grid (i.e. insert all the spheres)
        buildGrid(spheres, gridParams);

        if (verbose)
        {
            VR_INFO << "GridOfMedialSpheres::setup() finished." << endl;
        }

    }

    GridParameters GridOfMedialSpheres::determineGridParameters(vector<Vector3f> surfacePoints, vector<MedialSpherePtr> spheres, int gridConstant = 4)
    {
        if (verbose)
        {
            VR_INFO << "determineGridParameters() called" << endl;
            VR_INFO << "gridConstant: " << gridConstant << endl;
        }

        //determine the corners of axis-aligned bounding box of the surface points
        Vector3f minPoint;
        Vector3f maxPoint;
        findMinMax(surfacePoints, minPoint, maxPoint);

        if (verbose)
        {
            VR_INFO << "minPoint: " << StrOutHelpers::toString(minPoint) << endl;
            VR_INFO << "maxPoint: " << StrOutHelpers::toString(maxPoint) << endl;
        }

        //how many cells (for fast access)
        Vector3i numCells;
        float cellWidth;
        determineCellNumbers(surfacePoints, minPoint, maxPoint, gridConstant, numCells, cellWidth);
        GridParameters gridParams = GridParameters(numCells, minPoint, maxPoint, cellWidth);

        return gridParams;
    }

    void GridOfMedialSpheres::findMinMax(vector<Vector3f> points, Vector3f& minPoint, Vector3f& maxPoint)
    {
        //init
        minPoint = points.at(0);
        maxPoint = points.at(0);


        for (size_t i = 0; i < points.size(); i++)
        {
            //x
            if (points.at(i)(0) < minPoint(0))
            {
                minPoint(0) = points.at(i)(0);
            }
            else if (points.at(i)(0) > maxPoint(0))
            {
                maxPoint(0) = points.at(i)(0);
            }

            //y
            if (points.at(i)(1) < minPoint(1))
            {
                minPoint(1) = points.at(i)(1);
            }
            else if (points.at(i)(1) > maxPoint(1))
            {
                maxPoint(1) = points.at(i)(1);
            }

            //z
            if (points.at(i)(2) < minPoint(2))
            {
                minPoint(2) = points.at(i)(2);
            }
            else if (points.at(i)(2) > maxPoint(2))
            {
                maxPoint(2) = points.at(i)(2);
            }
        }
    }

    void GridOfMedialSpheres::determineCellNumbers(vector<Vector3f> points, Vector3f minPoint, Vector3f maxPoint, int gridConstant, Vector3i& numCells, float& cellWidth)
    {
        if (verbose)
        {
            VR_INFO << "determineGridParameters() called" << endl;
        }

        Vector3f diffVect = maxPoint - minPoint;
        float maxDist = diffVect.maxCoeff();
        float minDist = diffVect.minCoeff();

        float ratio = maxDist / minDist;
        int numPoints = points.size();

        //ACHTUNG: alter Algorithmus! TODO: Änderungen einpflegen!
        char mainDirection;

        //x is main dimension
        if (diffVect(0) == maxDist) //ACHTUNG: fuzzy floating point arithmetic... ?
        {
            mainDirection = 'x';
            //numCells(0) = ceil(ceil(ratio * (numPoints ** (1.0 / 3))) / gridConstant);
            numCells(0) = (int)ceil(ceil(ratio * pow((float)numPoints, (1.0f / 3.0f))) / gridConstant);
            cellWidth = maxDist / numCells(0);
            numCells(1) = (int)ceil(diffVect(1) / cellWidth);
            numCells(2) = (int)ceil(diffVect(2) / cellWidth);
        } //y is main dimension
        else if (diffVect(1) == maxDist) //ACHTUNG: fuzzy floating point arithmetic... ?
        {
            mainDirection = 'y';
            //numCells(1) = ceil(ceil(ratio * (num_points ** (1.0 / 3))) / k_factor);
            numCells(1) = (int)ceil(ceil(ratio * pow((float)numPoints, (1.0f / 3.0f))) / gridConstant);
            cellWidth = maxDist / numCells(1);
            numCells(0) = (int)ceil(diffVect(0) / cellWidth);
            numCells(2) = (int)ceil(diffVect(2) / cellWidth);

        } //z is main dimension
        else if (diffVect(2) == maxDist)
        {
            mainDirection = 'z';
            //numCells(2) = ceil(ceil(ratio * (numPoints ** (1.0 / 3))) / gridConstant);
            numCells(2) = (int)ceil(ceil(ratio * pow((float)numPoints, (1.0f / 3.0f))) / gridConstant);
            cellWidth = maxDist / numCells(2);
            numCells(0) = (int)ceil(diffVect(0) / cellWidth);
            numCells(1) = (int)ceil(diffVect(1) / cellWidth);
        }
        else    // Should never occur.
        {
            VR_ERROR << "GridOfMedialSpheres::determineCellNumbers(): Unexpected error!" << endl;
        }

        int numCellsTotal = (numCells(0) + 1) * (numCells(1) + 1) * (numCells(2) + 1);

        if (verbose)
        {
            VR_INFO << "main direction is: " << mainDirection << endl;
            VR_INFO << "numCells: " << numCells(0) << " " << numCells(1) << " " << numCells(2) << endl;
            VR_INFO << "numCellsTotal: " << numCellsTotal << " cellWidth: " << cellWidth << endl;
        }

    }

    void GridOfMedialSpheres::buildGrid(std::vector<MedialSpherePtr> spheres, GridParameters gridParams)
    {
        if (verbose)
        {
            VR_INFO << "buildGrid() called." << endl;
        }

        allocateGrid(gridParams.numCells);

        for (size_t i = 0; i < spheres.size(); i++)
        {
            insertSphere(spheres.at(i));
        }

        if (verbose)
        {
            VR_INFO << "buildGrid() finished." << endl;
        }
    }

    void GridOfMedialSpheres::allocateGrid(Eigen::Vector3i numCells)
    {
        if (verbose)
        {
            VR_INFO << "allocateGrid() called." << endl;
        }

        //numCells[i] + 1 ??
        int xdim = numCells(0);
        int ydim = numCells(1);
        int zdim = numCells(2);

        vector<MedialSpherePtr> v1D;
        vector< vector<MedialSpherePtr> > v2D;
        vector< vector< vector<MedialSpherePtr> > > v3D;

        v2D.reserve(zdim);

        for (size_t j = 0; j < v2D.capacity(); j++)
        {
            v2D.push_back(v1D);
        }

        v3D.reserve(ydim);

        for (size_t k = 0; k < v3D.capacity(); k++)
        {
            v3D.push_back(v2D);
        }

        grid.reserve(xdim);

        for (size_t l = 0; l < grid.capacity(); l++)
        {
            grid.push_back(v3D);
        }

        if (verbose)
        {
            VR_INFO << "allocateGrid() finished." << endl;
        }
    }

    void GridOfMedialSpheres::printDebugGridCells()
    {
        for (size_t x = 0; x < grid.size(); x++)
        {
            for (size_t y = 0; y < grid.at(x).size(); y++)
            {
                for (size_t z = 0; z < grid.at(x).at(y).size(); z++)
                {
                    VR_INFO << "Number of spheres in cell ["
                            << x << " " << y << " " << z << " " << "]: "
                            << grid.at(x).at(y).at(z).size() << endl;
                }

                VR_INFO << endl;
            }

            VR_INFO << endl;
        }

    }

    void GridOfMedialSpheres::insertSphere(MedialSpherePtr s)
    {
        Vector3i gridIndex = calcGridIndex(s->center);

        if (isGridIndexValid(gridIndex))
        {
            //update minRadius and maxRadius
            if (numberOfSpheres == 0)
            {
                minRadius = s->radius;
                maxRadius = s->radius;
            }
            else
            {
                if (s->radius > maxRadius)
                {
                    maxRadius = s->radius;
                }
                else if (s->radius < minRadius)
                {
                    minRadius = s->radius;
                }
            }

            //put sphere into the grid
            grid.at(gridIndex(0)).at(gridIndex(1)).at(gridIndex(2)).push_back(s);
        }
        else
        {
            if (verbose)
            {
                VR_WARNING << "INVALID grid index: " << gridIndex(0) << " " << gridIndex(1) << " " << gridIndex(2) << " --> Discard sphere (outside of surface points bounding box). " << endl;
            }
        }

    }

    Vector3i GridOfMedialSpheres::calcGridIndex(Vector3f sphereCenter)
    {
        float x = sphereCenter(0);
        float y = sphereCenter(1);
        float z = sphereCenter(2);

        float x_min = gridParameters.minPoint(0);
        float y_min = gridParameters.minPoint(1);
        float z_min = gridParameters.minPoint(2);
        float x_max = gridParameters.maxPoint(0);
        float y_max = gridParameters.maxPoint(1);
        float z_max = gridParameters.maxPoint(2);

        Vector3i gridIndex;
        gridIndex(0) = (int)floor(gridParameters.numCells(0) * (x - x_min) / (x_max - x_min));
        gridIndex(1) = (int)floor(gridParameters.numCells(1) * (y - y_min) / (y_max - y_min));
        gridIndex(2) = (int)floor(gridParameters.numCells(2) * (z - z_min) / (z_max - z_min));

        return gridIndex;
    }

    bool GridOfMedialSpheres::isGridIndexValid(Vector3i gridIndex)
    {
        //Check bounds
        if (gridIndex(0) > (gridParameters.numCells(0) - 1))
        {
            return false;
        }
        else if (gridIndex(1) > (gridParameters.numCells(1) - 1))
        {
            return false;
        }
        else if (gridIndex(2) > (gridParameters.numCells(2) - 1))
        {
            return false;
        }
        else if (gridIndex(0) < 0)
        {
            return false;
        }
        else if (gridIndex(1) < 0)
        {
            return false;
        }
        else if (gridIndex(2) < 0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    vector<MedialSpherePtr> GridOfMedialSpheres::getSpheresInNeighborhood(MedialSpherePtr s, float searchRadius)
    {
        if (verbose)
        {
            VR_INFO << "getSpheresInNeighborhood() called." << endl;
        }

        //Get all cube cells inside searchRadius
        Vector3i startGridIndex = calcGridIndex(s->center);
        int cubeRadiusToSearch = int(ceil(searchRadius / this->gridParameters.cellWidth));

        if (verbose)
        {
            VR_INFO << "startGridIndex: " << StrOutHelpers::toString(startGridIndex) << endl;
            VR_INFO << "searchRadius: " << searchRadius << endl;
            VR_INFO << "cellWidth: " << gridParameters.cellWidth << endl;
            VR_INFO << "cubeRadiusToSearch: " << cubeRadiusToSearch << endl;
        }

        //calc grid cell indices
        vector<Vector3i> cubeIndicesToSearch = computeCubeIndices(startGridIndex,
                                               this->gridParameters.maxGridIndex,
                                               cubeRadiusToSearch);

        //get spheres from the grid cells
        vector<MedialSpherePtr> spheresInCuboidNeighborhood = getSpheresFromGrid(cubeIndicesToSearch);


        //get all spheres inside searchRadius
        vector<MedialSpherePtr> spheresInSphericalNeighborhood =
            SphereHelpers::getSpheresInsideSearchRadius(spheresInCuboidNeighborhood,
                    s, searchRadius);


        return spheresInSphericalNeighborhood;
    }

    vector<Vector3i> GridOfMedialSpheres::computeCubeIndices(Vector3i seedIndex, Vector3i maxCoords, int cubeRadiusToSearch)
    {
        int maxIndexX = seedIndex(0) + cubeRadiusToSearch;
        int minIndexX = seedIndex(0) - cubeRadiusToSearch;

        if (maxIndexX > maxCoords(0))   //ACHTUNG: > oder >= ?
        {
            maxIndexX = maxCoords(0);    //ACHTUNG: evtl. maxCoords(0)-1 zuweisen?
        }

        if (minIndexX < 0)
        {
            minIndexX = 0;
        }

        int maxIndexY = seedIndex(1) + cubeRadiusToSearch;
        int minIndexY = seedIndex(1) - cubeRadiusToSearch;

        if (maxIndexY > maxCoords(1))   //ACHTUNG: > oder >= ?
        {
            maxIndexY = maxCoords(1);    //ACHTUNG: evtl. maxCoords(1)-1 zuweisen?
        }

        if (minIndexY < 0)
        {
            minIndexY = 0;
        }

        int maxIndexZ = seedIndex(2) + cubeRadiusToSearch;
        int minIndexZ = seedIndex(2) - cubeRadiusToSearch;

        if (maxIndexZ > maxCoords(2))   //ACHTUNG: > oder >= ?
        {
            maxIndexZ = maxCoords(2);    //ACHTUNG: evtl. maxCoords(2)-1 zuweisen?
        }

        if (minIndexZ < 0)
        {
            minIndexZ = 0;
        }

        vector<Vector3i> cubeIndices;
        Vector3i currentIndex;

        for (int i = minIndexX; i <= maxIndexX; i++)
            for (int j = minIndexY; j <= maxIndexY; j++)
                for (int k = minIndexZ; k <= maxIndexZ; k++)
                {
                    currentIndex << i, j, k;
                    cubeIndices.push_back(currentIndex);
                }

        return cubeIndices;
    }

    vector<MedialSpherePtr> GridOfMedialSpheres::getSpheresFromGrid(vector<Vector3i> gridIndices)
    {
        //    if (verbose)
        //    {
        //        VR_INFO << "get spheres from the following grid cells: " << endl;
        //        VR_INFO << StrOutHelpers::toString(gridIndices) << endl;
        //    }

        vector<MedialSpherePtr> vectorOfSpheres;

        for (size_t i = 0; i < gridIndices.size(); i++)
        {
            Vector3i currentIndex = gridIndices.at(i);
            vector<MedialSpherePtr>::iterator itStart = (grid.at(currentIndex(0)).at(currentIndex(1)).at(currentIndex(2))).begin();
            vector<MedialSpherePtr>::iterator itEnd = (grid.at(currentIndex(0)).at(currentIndex(1)).at(currentIndex(2))).end();
            vectorOfSpheres.insert(vectorOfSpheres.end(), itStart, itEnd);

            //        if (verbose)
            //        {
            //            VR_INFO << "currentIndex: "
            //                    << StrOutHelpers::toString(currentIndex) << endl;
            //            VR_INFO << "current spheres total: "
            //                    << vectorOfSpheres.size() << endl;
            //        }
        }

        //    VR_INFO << "getSpheresFromGrid() fetched " << vectorOfSpheres.size()
        //         << " spheres, leaving..." << endl;

        return vectorOfSpheres;

    }


}
