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
#include "GraspPlannerConfiguration.h"
#include <iostream>
using namespace std;

namespace GraspStudio
{

    GraspPlannerConfiguration::GraspPlannerConfiguration()
    {
        //constant for setting up the grid of medial spheres
        gridConstant = 4;

        //candidate grasp generation constants
        thresholdSymmetryAxis = 0.01f;   // do not touch!
        thresholdSymmetryPlane = 0.4f;   // do not touch!
        minimumObjectAngle = 120.0f;       // do not touch!

        //candidate grasp generation options
        minimumSphereRadiusRelative = 0.0;  //If >0.0, discard spheres that are small relative
        //to the biggest sphere in the object.
        //Useful to cancel noise and/or to discard
        //surface details.

        neighborhoodSearchRadius = 0.01f;    //Radius of the local neighborhood around a
        //medial sphere that should be analyzed during
        //candidate grasp generation.

        maxGraspDiameterOfCurrentRobotHand = -1.0;  //If !=-1, specifies the diameter of the
        //biggest sphere the hand can grasp.
        //In that case, no candidate grasps will be
        //generated for spheres bigger than that.

        generateFlippedCandidates = true;   //if true: for each candidate, generate a second
        //candidate with inverted hand orientation

        numberOfApproachDirectionsForLocalSymmetryAxis = 4;     //generate this number of approach
        //directions in equal angle
        //intervals around a local
        //symmetry axis

        fractionOfSpheresToAnalyze = 1.0;   //should all (filtered) medial spheres be used for
        //candidate grasp generation, or only a fraction of
        //them?

        stopAfterAnalyzingThisNumberOfSpheres = -1;  //analyze this number of spheres, then stop

        //visualization options
        //basic stuff
        drawSurfacePointCloud = false;
        drawMedialSpheresBeforeFiltering = false;
        drawMedialAxisPointCloudBeforeFiltering = false;
        drawMedialSpheresAfterFiltering = true;
        drawMedialAxisPointCloudAfterFiltering = true;

        //results of analysis
        drawNeighborhoods = true;
        drawNeighborhoodCenterOfGravity = true;
        drawNeighborhoodEigenvectors = true;
        drawNeighborhoodSearchRadius = false;
        drawScale = 0.003f;             //draw stuff this big
        drawPointSize = 0.001f;
        drawCandidateGrasps = true;

        //debug output
        printMedialSpheres = false;
        printNeighborhoods = false;
        printCandidateGrasps = false;

    }

    void GraspPlannerConfiguration::printDebug()
    {
        cout << "=== GraspPlannerConfiguration: ===" << endl;

        cout << "thresholdSymmetryAxis: " << thresholdSymmetryAxis << endl;
        cout << "thresholdSymmetryPlane: " << thresholdSymmetryPlane << endl;

        cout << "minimumObjectAngle: " << minimumObjectAngle << endl;
        cout << "minimumSphereRadiusRelative: " << minimumSphereRadiusRelative << endl;

        cout << "neighborhoodSearchRadius: " << neighborhoodSearchRadius << endl;
        cout << "maxGraspDiameterOfCurrentRobotHand: "
             << maxGraspDiameterOfCurrentRobotHand << endl;

        cout << "generateFlippedCandidates: " << generateFlippedCandidates << endl;
        cout << "numberOfApproachDirectionsForLocalSymmetryAxis: "
             << numberOfApproachDirectionsForLocalSymmetryAxis << endl;

        cout << "fractionOfSpheresToAnalyze: " << fractionOfSpheresToAnalyze << endl;
        cout << "stopAfterAnalyzingThisNumberOfSpheres: "
             << stopAfterAnalyzingThisNumberOfSpheres << endl;

        cout << "drawSurfacePointCloud:" << drawSurfacePointCloud << endl;
        cout << "drawMedialSpheresBeforeFiltering: "
             << drawMedialSpheresBeforeFiltering << endl;
        cout << "drawMedialAxisPointCloudBeforeFiltering: "
             << drawMedialAxisPointCloudBeforeFiltering << endl;
        cout << "drawMedialSpheresAfterFiltering: "
             << drawMedialSpheresAfterFiltering << endl;
        cout << "drawMedialAxisPointCloudAfterFiltering: "
             << drawMedialAxisPointCloudAfterFiltering << endl;

        cout << "drawNeighborhoods: " << drawNeighborhoods << endl;
        cout << "drawNeighborhoodCenterOfGravity: "
             << drawNeighborhoodCenterOfGravity << endl;
        cout << "drawNeighborhoodEigenvectors: "
             << drawNeighborhoodEigenvectors << endl;
        cout << "drawNeighborhoodSearchRadius: "
             << drawNeighborhoodSearchRadius << endl;
        cout << "drawScale: " << drawScale << endl;
        cout << "drawPointSize: " << drawPointSize << endl;
        cout << "drawCandidateGrasps: " << drawCandidateGrasps << endl;

        cout << "printMedialSpheres: " << printMedialSpheres << endl;
        cout << "printNeighborhoods: " << printNeighborhoods << endl;
        cout << "printCandidateGrasps: " << printCandidateGrasps << endl;

    }
}
