#ifndef GRASPPLANNERCONFIGURATION_H
#define GRASPPLANNERCONFIGURATION_H

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

#include <iostream>

#include "../../GraspStudio.h"

namespace GraspStudio
{
    class GraspPlannerConfiguration;
    typedef boost::shared_ptr<GraspPlannerConfiguration> GraspPlannerConfigurationPtr;

    class GRASPSTUDIO_IMPORT_EXPORT GraspPlannerConfiguration
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GraspPlannerConfiguration();

        void printDebug();

        int gridConstant;

        //some constants for candidate grasp generation
        float thresholdSymmetryAxis;
        float thresholdSymmetryPlane;
        float minimumObjectAngle;
        float minimumSphereRadiusRelative;
        float neighborhoodSearchRadius;
        float maxGraspDiameterOfCurrentRobotHand;

        bool generateFlippedCandidates;
        int numberOfApproachDirectionsForLocalSymmetryAxis;

        float fractionOfSpheresToAnalyze;
        int stopAfterAnalyzingThisNumberOfSpheres;


        //visualization options
        //basic stuff
        bool drawSurfacePointCloud;
        bool drawMedialSpheresBeforeFiltering;
        bool drawMedialAxisPointCloudBeforeFiltering;
        bool drawMedialSpheresAfterFiltering;
        bool drawMedialAxisPointCloudAfterFiltering;

        //results of analysis
        bool drawNeighborhoods;
        bool drawNeighborhoodCenterOfGravity;
        bool drawNeighborhoodEigenvectors;
        bool drawNeighborhoodSearchRadius;
        float drawScale;
        float drawPointSize;
        bool drawCandidateGrasps;

        //debug output
        bool printMedialSpheres;
        bool printNeighborhoods;
        bool printCandidateGrasps;

    };

}
#endif // GRASPPLANNERCONFIGURATION_H
