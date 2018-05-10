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
#ifndef CANDIDATEGRASPGENERATOR_H
#define CANDIDATEGRASPGENERATOR_H


#include <vector>
#include "../../GraspStudio.h"
#include "CandidateGrasp.h"
#include "LocalNeighborhood.h"
#include "GraspPlannerConfiguration.h"

namespace GraspStudio
{

    class CandidateGraspGenerator;
    typedef boost::shared_ptr<CandidateGraspGenerator> CandidateGraspGeneratorPtr;

    class GRASPSTUDIO_IMPORT_EXPORT CandidateGraspGenerator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CandidateGraspGenerator();
        CandidateGraspGenerator(bool verbose);

        std::vector<CandidateGraspPtr> generateCandidateGrasps(LocalNeighborhoodPtr nbhd, GraspPlannerConfigurationPtr gpConfig);
        std::vector<CandidateGraspPtr> generateSymmetryAxisOrthogonalCandidateGrasps(LocalNeighborhoodPtr nbhd, GraspPlannerConfigurationPtr gpConfig);
        std::vector<CandidateGraspPtr> generateSymmetryPlaneParallelCandidateGrasps(LocalNeighborhoodPtr nbhd, GraspPlannerConfigurationPtr gpConfig);

    private:
        std::vector<Eigen::Vector3f> generatePointsOnCircle(Eigen::Vector3f circleCenter, Eigen::Vector3f rotationAxis, int numberOfPoints, float radius);

        bool verbose;

    };

}
#endif // CANDIDATEGRASPGENERATOR_H
