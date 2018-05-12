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
#include "LocalNeighborhood.h"

namespace GraspStudio
{

    LocalNeighborhood::LocalNeighborhood()
    {
    }

    LocalNeighborhood::LocalNeighborhood(MedialSpherePtr seedSphere, std::vector<MedialSpherePtr>& spheresInNeighborhood, float neighborhoodRadius)
    {
        centerSphere = seedSphere;
        center = seedSphere->center;
        spheres = spheresInNeighborhood;
        radius = neighborhoodRadius;
        numberOfSpheres = spheres.size();

        if (numberOfSpheres > 1)
        {
            isEmpty = false;
        }
        else
        {
            isEmpty = true;
        }

        hasBeenAnalyzed = false;
    }

    void LocalNeighborhood::analyze()
    {
        determineEigenvectorsAndEigenvalues_viaSVD();
        computeCenterOfGravity();

        hasBeenAnalyzed = true;
    }

    void LocalNeighborhood::determineEigenvectorsAndEigenvalues_viaSVD()
    {
        std::vector<Eigen::Vector3f> sphereCenters = SphereHelpers::getSphereCenters(spheres);
        Eigen::MatrixXf sphereCentersAsMatrix = SphereHelpers::toMatrix_3xN(sphereCenters);

        //normalize the data
        Eigen::Vector3f meanVect = sphereCentersAsMatrix.rowwise().mean();
        Eigen::MatrixXf sphereCentersAsMatrixNormalized = sphereCentersAsMatrix.colwise() - meanVect;

        Eigen::JacobiSVD<Eigen::MatrixXf> svd(sphereCentersAsMatrixNormalized,
                                              Eigen::ComputeThinU | Eigen::ComputeThinV);

        //    std::cout << "Its singular values are: " << std::endl << svd.singularValues() << std::endl;
        //    std::cout << "Its left singular vectors are the columns of the thin U matrix: "
        //                 << std::endl << svd.matrixU() << std::endl;

        eigenvalue1 = svd.singularValues()[0] * svd.singularValues()[0];
        eigenvalue2 = svd.singularValues()[1] * svd.singularValues()[1];

        eigenvector1 = svd.matrixU().block<3, 1>(0, 0);
        eigenvector2 = svd.matrixU().block<3, 1>(0, 1);

        ratioOfEigenvalues = eigenvalue2 / eigenvalue1;
    }

    //void LocalNeighborhood::determineEigenvectorsAndEigenvalues_viaCovMatrix()
    //{
    //    std::vector<Eigen::Vector3f> sphereCenters = SphereHelpers::getSphereCenters(spheres);
    //    Eigen::MatrixXf sphereCentersAsMatrix = SphereHelpers::toMatrix_3xN(sphereCenters);

    //    //normalize the data
    //    Eigen::Vector3f meanVect = sphereCentersAsMatrix.rowwise().mean();
    //    Eigen::MatrixXf sphereCentersAsMatrixNormalized = sphereCentersAsMatrix.colwise() - meanVect;

    //    Eigen::MatrixXf cov = sphereCentersAsMatrixNormalized * sphereCentersAsMatrixNormalized.transpose();

    //    Eigen::EigenSolver<Eigen::MatrixXf> es(cov,true);

    ////    std::cout << "eigenvalues: " << es.eigenvalues() << std::endl;
    ////    std::cout << "eigenvectors: " << es.eigenvectors() << std::endl;

    //    eigenvalue1 = std::real(es.eigenvalues()[0]);
    //    eigenvalue2 = std::real(es.eigenvalues()[1]);

    //    if (eigenvalue1 > eigenvalue2)
    //    {
    //        //everything OK
    //        eigenvector1 << std::real(es.eigenvectors().col(0)[0]),
    //                      std::real(es.eigenvectors().col(0)[1]),
    //                      std::real(es.eigenvectors().col(0)[2]);
    //        eigenvector2 << std::real(es.eigenvectors().col(1)[0]),
    //                      std::real(es.eigenvectors().col(1)[1]),
    //                      std::real(es.eigenvectors().col(1)[2]);
    //    }
    //    else
    //    {
    //        //swap stuff
    //        eigenvalue1 = std::real(es.eigenvalues()[1]);
    //        eigenvalue2 = std::real(es.eigenvalues()[0]);
    //        eigenvector1 << std::real(es.eigenvectors().col(1)[0]),
    //                      std::real(es.eigenvectors().col(1)[1]),
    //                      std::real(es.eigenvectors().col(1)[2]);
    //        eigenvector2 << std::real(es.eigenvectors().col(0)[0]),
    //                      std::real(es.eigenvectors().col(0)[1]),
    //                      std::real(es.eigenvectors().col(0)[2]);
    //    }

    ////    std::cout << "eigenvalue1: " << eigenvalue1 << " eigenvalue2: " << eigenvalue2 << std::endl;
    ////    std::cout << "eigenvector1: " << eigenvector1 << std::endl;
    ////    std::cout << "eigenvector2: " << eigenvector2 << std::endl;

    //    //    std::cout << "eigenvectors: " << es.eigenvectors() << std::endl;

    //    ratioOfEigenvalues = eigenvalue2 / eigenvalue1;

    //}

    void LocalNeighborhood::computeCenterOfGravity()
    {
        std::vector<Eigen::Vector3f> sphereCenters = SphereHelpers::getSphereCenters(spheres);
        Eigen::MatrixXf sphereCentersAsMatrix = SphereHelpers::toMatrix_3xN(sphereCenters);

        centerOfGravity = sphereCentersAsMatrix.rowwise().mean();
    }

    void LocalNeighborhood::printDebug()
    {
        std::cout << "=== LocalNeighborhood ===" << std::endl;
        std::cout << "center: " << StrOutHelpers::toString(center) << std::endl;
        std::cout << "radius: " << radius << std::endl;
        std::cout << "numberOfSpheres: " << numberOfSpheres << std::endl;
        std::cout << "isEmpty: " << isEmpty << std::endl;
        std::cout << "hasBeenAnalyzed: " << hasBeenAnalyzed << std::endl;

        if (hasBeenAnalyzed)
        {
            std::cout << "eigenvector1: " << StrOutHelpers::toString(eigenvector1) << std::endl;
            std::cout << "eigenvector2: " << StrOutHelpers::toString(eigenvector2) << std::endl;
            std::cout << "eigenvalue1: " << eigenvalue1 << std::endl;
            std::cout << "eigenvalue2: " << eigenvalue2 << std::endl;
            std::cout << "ratioOfEigenvalues: " << ratioOfEigenvalues << std::endl;
            std::cout << "centerOfGravity: " << StrOutHelpers::toString(centerOfGravity) << std::endl;
        }
    }

    bool LocalNeighborhood::isValid()
    {
        if (ratioOfEigenvalues < 0.0)
        {
            VR_WARNING << "WARNING: LocalNeighborhood::isValid(): ratioOfEigenvalues negative! Discarding this local neighborhood!" << std::endl;
            printDebug();
            return false;
        }

        if (eigenvalue1 <= 0.0)
        {
            VR_WARNING << "WARNING: LocalNeighborhood::isValid(): eigenvalue1 zero/negative! Discarding this local neighborhood!" << std::endl;
            printDebug();
            return false;
        }

        if (eigenvalue2 < 0.0)
        {
            VR_WARNING << "WARNING: LocalNeighborhood::isValid(): eigenvalue2 negative! Discarding this local neighborhood!" << std::endl;
            printDebug();
            return false;
        }

        return true;
    }

}
