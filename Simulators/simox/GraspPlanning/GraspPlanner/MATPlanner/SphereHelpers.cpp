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
#include "SphereHelpers.h"
using namespace Eigen;

namespace GraspStudio
{

    SphereHelpers::SphereHelpers()
    {
    }


    float SphereHelpers::findMinSphereRadius(std::vector<MedialSpherePtr>& spheres)
    {
        float minRadius = 1000000.0;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            if (spheres.at(i)->radius < minRadius)
            {
                minRadius = spheres.at(i)->radius;
            }
        }

        return minRadius;
    }

    float SphereHelpers::findMaxSphereRadius(std::vector<MedialSpherePtr>& spheres)
    {
        float maxRadius = -1.0;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            if (spheres.at(i)->radius > maxRadius)
            {
                maxRadius = spheres.at(i)->radius;
            }
        }

        return maxRadius;
    }

    float SphereHelpers::findMinObjectAngleDegrees(std::vector<MedialSpherePtr>& spheres)
    {
        float minObjectAngleDegrees = 360.0;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            if (spheres.at(i)->objectAngle < minObjectAngleDegrees)
            {
                minObjectAngleDegrees = spheres.at(i)->objectAngle;
            }
        }

        return minObjectAngleDegrees;
    }

    float SphereHelpers::findMaxObjectAngleDegrees(std::vector<MedialSpherePtr>& spheres)
    {
        float maxObjectAngleDegrees = -1.0;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            if (spheres.at(i)->objectAngle > maxObjectAngleDegrees)
            {
                maxObjectAngleDegrees = spheres.at(i)->objectAngle;
            }
        }

        return maxObjectAngleDegrees;
    }

    void SphereHelpers::scaleMedialSpheres(std::vector<MedialSpherePtr>& spheres, float scaleFactor)
    {
        for (size_t i = 0; i < spheres.size(); i++)
        {
            spheres.at(i)->scale(scaleFactor);
        }
    }

    void SphereHelpers::selectSpheresWithMinRadius(std::vector<MedialSpherePtr>& spheres, float minRadius)
    {
        std::vector<MedialSpherePtr> resultSpheres;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            if (spheres.at(i)->radius > minRadius)
            {
                resultSpheres.push_back(spheres.at(i));
            }
        }

        spheres = resultSpheres;
    }

    void SphereHelpers::selectSpheresWithMaxRadius(std::vector<MedialSpherePtr>& spheres, float maxRadius)
    {
        std::vector<MedialSpherePtr> resultSpheres;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            if (spheres.at(i)->radius < maxRadius)
            {
                resultSpheres.push_back(spheres.at(i));
            }
        }

        spheres = resultSpheres;
    }

    void SphereHelpers::selectSpheresWithMinObjectAngle(std::vector<MedialSpherePtr>& spheres, float minObjectAngle)
    {
        std::vector<MedialSpherePtr> resultSpheres;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            if (spheres.at(i)->objectAngle > minObjectAngle)
            {
                resultSpheres.push_back(spheres.at(i));
            }
        }

        spheres = resultSpheres;
    }

    void SphereHelpers::selectSpheresWithMaxObjectAngle(std::vector<MedialSpherePtr>& spheres, float maxObjectAngle)
    {
        std::vector<MedialSpherePtr> resultSpheres;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            if (spheres.at(i)->objectAngle < maxObjectAngle)
            {
                resultSpheres.push_back(spheres.at(i));
            }
        }

        spheres = resultSpheres;
    }

    void SphereHelpers::selectSpheresWithConstraints(std::vector<MedialSpherePtr>& spheres, SphereConstraintsSet constraints)
    {
        if (constraints.minObjectAngle > -1)
        {
            selectSpheresWithMinObjectAngle(spheres, constraints.minObjectAngle);
        }

        if (constraints.maxObjectAngle > -1)
        {
            selectSpheresWithMaxObjectAngle(spheres, constraints.maxObjectAngle);
        }

        if (constraints.minRadius > -1)
        {
            selectSpheresWithMinRadius(spheres, constraints.minRadius);
        }

        if (constraints.maxRadius > -1)
        {
            selectSpheresWithMaxRadius(spheres, constraints.maxRadius);
        }
    }

    SphereConstraintsSet SphereHelpers::getSphereConstraintsSet(std::vector<MedialSpherePtr>& spheres)
    {
        float minRadius = findMinSphereRadius(spheres);
        float maxRadius = findMaxSphereRadius(spheres);
        float minObjectAngle = findMinObjectAngleDegrees(spheres);
        float maxObjectAngle = findMaxObjectAngleDegrees(spheres);
        int numSpheres = spheres.size();

        SphereConstraintsSet scs(minRadius, maxRadius, minObjectAngle, maxObjectAngle, numSpheres);

        return scs;
    }

    void SphereHelpers::filterSpheres(std::vector<MedialSpherePtr>& spheres, float minObjAngle, float minRadiusRelative, bool verbose)
    {
        if (verbose)
        {
            std::cout << "SphereHelpers::filterSpheres() called." << std::endl;
        }

        SphereConstraintsSet scs0;
        scs0 = SphereHelpers::getSphereConstraintsSet(spheres);

        SphereConstraintsSet scs1;
        scs1.minObjectAngle = minObjAngle;
        scs1.minRadius = minRadiusRelative * scs0.maxRadius;

        if (verbose)
        {
            std::cout << "\n scs: " << std::endl;
            scs1.printDebug();
        }

        SphereHelpers::selectSpheresWithConstraints(spheres, scs1);

        SphereConstraintsSet scs2;
        scs2 = SphereHelpers::getSphereConstraintsSet(spheres);

        if (verbose)
        {
            scs2.printDebug();
            std::cout << "SphereHelpers::filterSpheres() done." << std::endl;
        }

    }

    std::vector<MedialSpherePtr> SphereHelpers::getSpheresInsideSearchRadius(std::vector<MedialSpherePtr>& spheresToBeFiltered, MedialSpherePtr querySphere, float searchRadius)
    {
        std::vector<MedialSpherePtr> spheresInsideSearchRadius;

        for (size_t i = 0; i < spheresToBeFiltered.size(); i++)
        {
            Eigen::Vector3f distVect = querySphere->center - spheresToBeFiltered.at(i)->center;
            float distance = distVect.norm();

            //std::cout << "(getSpheresInsideSearchRadius()) distance: " << distance << std::endl;
            if (distance <= searchRadius)
            {
                //std::cout << "Adding this sphere." << std::endl;
                spheresInsideSearchRadius.push_back(spheresToBeFiltered.at(i));
            }
        }

        //    std::cout << "getSpheresInsideSearchRadius() got " << spheresInsideSearchRadius.size()
        //              << " spheres, leaving... " << std::endl;

        return spheresInsideSearchRadius;
    }

    std::vector<Eigen::Vector3f> SphereHelpers::getSphereCenters(std::vector<MedialSpherePtr>& spheres)
    {
        std::vector<Eigen::Vector3f> sphereCenters;

        for (size_t i = 0; i < spheres.size(); i++)
        {
            sphereCenters.push_back(spheres.at(i)->center);
        }

        return sphereCenters;
    }


    SphereConstraintsSet::SphereConstraintsSet()
    {
        minRadius = -1;
        maxRadius = -1;
        minObjectAngle = -1;
        maxObjectAngle = -1;

        numberOfSpheres = -1;
    }

    SphereConstraintsSet::SphereConstraintsSet(float r_min, float r_max, float alpha_min, float alpha_max, int numSpheres)
    {
        minRadius = r_min;
        maxRadius = r_max;
        minObjectAngle = alpha_min;
        maxObjectAngle = alpha_max;

        numberOfSpheres = numSpheres;
    }

    void SphereConstraintsSet::printDebug()
    {
        std::cout << "=== SphereConstraintsSet ===" << std::endl;
        std::cout << "minRadius: " << minRadius << std::endl;
        std::cout << "maxRadius: " << maxRadius << std::endl;
        std::cout << "minObjectAngle: " << minObjectAngle << std::endl;
        std::cout << "maxObjectAngle: " << maxObjectAngle << std::endl;
        std::cout << "numberOfSpheres: " << numberOfSpheres << std::endl;
    }


    float SphereHelpers::calcAngleBetweenTwoVectorsRad(Vector3f a, Vector3f b)
    {
        float length_a = sqrt(a.dot(a));
        float length_b = sqrt(b.dot(b));

        float denominator = length_a * length_b;
        float numerator = a.dot(b);

        return acos(numerator / denominator);
    }

    float SphereHelpers::calcAngleBetweenTwoVectorsDeg(Vector3f a, Vector3f b)
    {
        float angleRad = calcAngleBetweenTwoVectorsRad(a, b);
        return (angleRad * 180.0f) / (float)M_PI; //TODO: PI woher?

    }
    MatrixXf SphereHelpers::toMatrix_3xN(std::vector<Vector3f> points)
    {
        MatrixXf m;
        m.resize(3, points.size());

        for (size_t i = 0; i < points.size(); i++)
        {
            m.block<3, 1>(0, i) = points.at(i);
        }

        return m;
    }
}
