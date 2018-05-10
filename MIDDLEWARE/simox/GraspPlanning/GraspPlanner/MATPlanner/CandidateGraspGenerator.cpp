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
#include "CandidateGraspGenerator.h"

using namespace std;
namespace GraspStudio
{


    CandidateGraspGenerator::CandidateGraspGenerator()
    {
        verbose = false;
    }

    CandidateGraspGenerator::CandidateGraspGenerator(bool verbose)
    {
        this->verbose = verbose;
    }


    std::vector<CandidateGraspPtr> CandidateGraspGenerator::generateCandidateGrasps(LocalNeighborhoodPtr nbhd, GraspPlannerConfigurationPtr gpConfig)
    {
        //TODO: (maybe) don't generate candidate grasps for very small spheres

        std::vector<CandidateGraspPtr> candidates;

        if ((2.0 * nbhd->centerSphere->radius > gpConfig->maxGraspDiameterOfCurrentRobotHand)
            && (gpConfig->maxGraspDiameterOfCurrentRobotHand > 0.0))
            //sphere too big to grasp
        {
            if (verbose)
                VR_INFO << "CandidateGraspGenerator::generateCandidateGrasps(): sphere radius "
                        << nbhd->centerSphere->radius
                        << " too big to grasp with current robot hand, maxGraspDiameterOfCurrentRobotHand: "
                        << gpConfig->maxGraspDiameterOfCurrentRobotHand << endl;

            return candidates;
        }

        if (nbhd->isEmpty)
        {
            return candidates;
        }

        if (nbhd->ratioOfEigenvalues <= gpConfig->thresholdSymmetryAxis)
        {
            candidates = generateSymmetryAxisOrthogonalCandidateGrasps(nbhd, gpConfig);
        }
        else
        {
            if (nbhd->ratioOfEigenvalues <= gpConfig->thresholdSymmetryPlane)
            {
                candidates = generateSymmetryPlaneParallelCandidateGrasps(nbhd, gpConfig);
            }
            else
            {
                if (verbose)
                    VR_INFO << "CandidateGraspGenerator::generateCandidateGrasps(): ratioOfEigenvalues: "
                            << nbhd->ratioOfEigenvalues
                            << " -> target sphere located inside plane, skip generating approach directions."
                            << endl;
            }
        }

        return candidates;
    }

    std::vector<CandidateGraspPtr> CandidateGraspGenerator::generateSymmetryAxisOrthogonalCandidateGrasps(LocalNeighborhoodPtr nbhd, GraspPlannerConfigurationPtr gpConfig)
    {
        std::vector<CandidateGraspPtr> candidates;

        Eigen::Vector3f symAxisDirection = nbhd->eigenvector1;
        int numApproachDirections = gpConfig->numberOfApproachDirectionsForLocalSymmetryAxis;

        float radius = 1;
        std::vector<Eigen::Vector3f> pointsOnCircle = generatePointsOnCircle(nbhd->centerSphere->center,
                symAxisDirection, numApproachDirections, radius);

        for (size_t i = 0; i < pointsOnCircle.size(); i++)
        {
            CandidateGraspPtr cg = CandidateGraspPtr(new CandidateGrasp);
            cg->graspTargetPoint = nbhd->centerSphere->center;
            cg->handApproachDirection = pointsOnCircle.at(i) - nbhd->centerSphere->center;
            cg->handOrientation = symAxisDirection;
            cg->candidateGraspType = "symmetryAxisOrthogonal";

            cg->medialSphere = nbhd->centerSphere;
            cg->localNeighborhood = nbhd;
            candidates.push_back(cg);
        }

        if (gpConfig->generateFlippedCandidates)
        {
            for (size_t i = 0; i < pointsOnCircle.size(); i++)
            {
                CandidateGraspPtr cg = CandidateGraspPtr(new CandidateGrasp);
                cg->graspTargetPoint = nbhd->centerSphere->center;
                cg->handApproachDirection = pointsOnCircle.at(i) - nbhd->centerSphere->center;
                cg->handOrientation = -1 * symAxisDirection;
                cg->candidateGraspType = "symmetryAxisOrthogonal";

                cg->medialSphere = nbhd->centerSphere;
                cg->localNeighborhood = nbhd;
                candidates.push_back(cg);
            }
        }

        return candidates;
    }

    std::vector<CandidateGraspPtr> CandidateGraspGenerator::generateSymmetryPlaneParallelCandidateGrasps(LocalNeighborhoodPtr nbhd, GraspPlannerConfigurationPtr gpConfig)
    {
        std::vector<CandidateGraspPtr> candidates;

        Eigen::Vector3f planeDirection0 = nbhd->eigenvector1;
        Eigen::Vector3f planeDirection1 = nbhd->eigenvector2;

        Eigen::Vector3f temporaryApproachDirection = planeDirection1;
        Eigen::Vector3f finalApproachDirection;

        Eigen::Vector3f directionToCoG = nbhd->centerOfGravity - nbhd->center;
        float angleDeg = SphereHelpers::calcAngleBetweenTwoVectorsDeg(temporaryApproachDirection,
                         directionToCoG);

        if (angleDeg <= 90)
        {
            finalApproachDirection = -1 * temporaryApproachDirection;
        }
        else
        {
            finalApproachDirection = temporaryApproachDirection;
        }

        CandidateGraspPtr cg = CandidateGraspPtr(new CandidateGrasp);
        cg->graspTargetPoint = nbhd->centerSphere->center;
        cg->handApproachDirection = finalApproachDirection;
        cg->handOrientation = planeDirection0;
        cg->candidateGraspType = "symmetryPlaneParallel";

        cg->medialSphere = nbhd->centerSphere;
        cg->localNeighborhood = nbhd;
        candidates.push_back(cg);

        if (gpConfig->generateFlippedCandidates)
        {
            CandidateGraspPtr cgFlipped = CandidateGraspPtr(new CandidateGrasp);
            cgFlipped->graspTargetPoint = nbhd->centerSphere->center;
            cgFlipped->handApproachDirection = finalApproachDirection;
            cgFlipped->handOrientation = -1 * planeDirection0;
            cgFlipped->candidateGraspType = "symmetryPlaneParallel";

            cgFlipped->medialSphere = nbhd->centerSphere;
            cgFlipped->localNeighborhood = nbhd;
            candidates.push_back(cgFlipped);
        }

        return candidates;
    }


    std::vector<Eigen::Vector3f> CandidateGraspGenerator::generatePointsOnCircle(Eigen::Vector3f circleCenter, Eigen::Vector3f rotationAxis, int numberOfPoints, float radius)
    {
        std::vector<Eigen::Vector3f> pointsOnCircle;

        Eigen::Vector3f xAxis(1, 0, 0);
        Eigen::Vector3f yAxis(0, 1, 0);
        Eigen::Vector3f zAxis(0, 0, 1);

        Eigen::Vector3f xAxisNew;
        Eigen::Vector3f yAxisNew;
        Eigen::Vector3f zAxisNew;

        //calculate a new coordinate system where rotationAxis is the new z axis.
        if ((zAxis - rotationAxis).norm() > 1e-12)
        {
            xAxisNew = zAxis.cross(rotationAxis);
            yAxisNew = xAxisNew.cross(rotationAxis);
            zAxisNew = rotationAxis;

            float xAxisNewNorm = xAxisNew.norm();
            float yAxisNewNorm = yAxisNew.norm();
            float zAxisNewNorm = zAxisNew.norm();

            //dangerous (Norm 0)
            if ((xAxisNewNorm < 1e-12) || (yAxisNewNorm < 1e-12) || (zAxisNewNorm < 1e-12) || verbose)
            {
                VR_WARNING << "-----> WARNING: generatePointsOnCircle(): norms of new coordinate frame axes are close to zero; skipping..." << endl;
            }
            else
            {
                xAxisNew = xAxisNew / xAxisNewNorm;
                yAxisNew = yAxisNew / yAxisNewNorm;
                zAxisNew = zAxisNew / zAxisNewNorm;
            }

            if (verbose)
            {
                VR_INFO << "generatePointsOnCircle(): NEW coord system" << endl;
            }

        }
        else
        {
            xAxisNew = xAxis;
            yAxisNew = yAxis;
            zAxisNew = zAxis;

            if (verbose)
            {
                VR_INFO << "generatePointsOnCircle(): did not change coord system." << endl;
            }
        }

        //calculate points on a circle around rotationAxis
        std::vector<float> polarAngles;
        float angleStepRad = (float)(2.0f * M_PI / (float)numberOfPoints);

        for (int k = 0; k < numberOfPoints; k++)
        {
            polarAngles.push_back(k * angleStepRad);
        }

        for (int i = 0; i < numberOfPoints; i++)
        {
            float phi = polarAngles.at(i);
            Eigen::Vector3f pointCoordsCartesian = circleCenter + radius * xAxisNew * cos(phi)
                                                   + radius * yAxisNew * sin(phi);
            pointsOnCircle.push_back(pointCoordsCartesian);
        }

        if (verbose)
        {
            VR_INFO << "generatePointsOnCircle(): points on circle:" << endl;
            VR_INFO << StrOutHelpers::toString(pointsOnCircle) << endl;
        }

        return pointsOnCircle;
    }

}

