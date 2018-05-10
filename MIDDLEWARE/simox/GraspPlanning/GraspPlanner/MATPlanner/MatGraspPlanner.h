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
* @author     Markus Przybylski, Nikolaus Vahrenkamp
* @copyright  2013 H2T,KIT
*             GNU Lesser General Public License
*
*/
#ifndef __MAT_GRASP_PLANNER_H__
#define __MAT_GRASP_PLANNER_H__

#include "../../GraspStudio.h"
#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/GraspPlanner/GraspPlanner.h>
#include <GraspPlanning/GraspQuality/GraspQualityMeasure.h>
#include "../../ExternalDependencies/powercrust/powercrust.h"


#include "MedialSphere.h"
#include "Converter.h"
#include "SphereHelpers.h"
#include "LocalNeighborhood.h"
#include "CandidateGraspGenerator.h"
#include "CandidateGraspTester.h"
#include "GridOfMedialSpheres.h"



namespace GraspStudio
{
    /*!
    *
    *
    * A general grasp planning class that utilizes ApprachMovementGenerator for generating grasp hypothesis and
    * GraspQualityMeasure to score them.
    *
    */
    class MatGraspPlanner;
    typedef boost::shared_ptr<GraspStudio::MatGraspPlanner> MatGraspPlannerPtr;

    class GRASPSTUDIO_IMPORT_EXPORT MatGraspPlanner : public GraspPlanner
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            \param graspSet All planned grasps are added to this GraspSet.
            \param eef
            \param graspQuality The quality of generated grasps is evaluated with this object
            \param gpConfig If not given, a standard setup is used.
            \param minQuality The quality that must be achieved at minimum by the GraspQualityMesurement module
            \param forceClosure When true, only force closure grasps are generated.
        */
        MatGraspPlanner(VirtualRobot::GraspSetPtr graspSet,
                        VirtualRobot::RobotPtr robot,
                        VirtualRobot::EndEffectorPtr eef,
                        GraspStudio::GraspQualityMeasureWrenchSpacePtr graspQuality,
                        GraspPlannerConfigurationPtr gpConfig = GraspPlannerConfigurationPtr(),
                        float minQuality = 0.0f,
                        bool forceClosure = true,
                        bool verbose = false);

        // destructor
        virtual ~MatGraspPlanner();


        bool init();

        /*!
            Creates new grasps.
            \param nrGrasps The number of grasps to be planned.
            \param timeOutMS The time out in milliseconds. Planning is stopped when this time is exceeded. Disabled when zero.
            \return Number of generated grasps.
        */
        virtual int plan(int nrGrasps, int timeOutMS = 0, VirtualRobot::SceneObjectSetPtr obstacles = VirtualRobot::SceneObjectSetPtr());


        std::vector<MedialSpherePtr> getMedialSpheres();
        std::vector<MedialSpherePtr> getMedialSpheresFiltered();

        std::vector<LocalNeighborhoodPtr> getLocalNeighborhoods();
        std::vector<CandidateGraspPtr> getCandidateGrasps();

        std::vector<Eigen::Vector3f> getSurfacePoints();

        /*!

        */
        void testCandidate(CandidateGraspPtr candidate);
    protected:

        void executePowercrust();
        std::vector<Eigen::Vector3f> getSurfacePoints(float scaling, bool checkForDoubledEntries = false);

        std::vector<CandidateGraspPtr> computeCandidates(MedialSpherePtr ms);
        bool computeAllCandidates();

        bool timeout();
        std::vector<Eigen::Vector3f> surfacePoints;
        GraspStudio::PowerCrust::PowerCrustPtr pc;

        VirtualRobot::GraspPtr planGrasp();

        VirtualRobot::SceneObjectPtr object;
        VirtualRobot::EndEffectorPtr eef;

        bool initialized;

        clock_t startTime;
        int timeOutMS;

        GraspStudio::GraspQualityMeasureWrenchSpacePtr graspQuality;
        float minQuality;
        bool forceClosure;

        //Grasp planner components:
        GridOfMedialSpheresPtr goms;
        CandidateGraspGeneratorPtr cgGenerator;
        CandidateGraspTesterPtr cgTester;
        GraspPlannerConfigurationPtr gpConfig;

        //some data structures fro the grasp planner:
        std::vector<MedialSpherePtr> medialSpheres;
        std::vector<MedialSpherePtr> medialSpheresFiltered;
        std::vector<LocalNeighborhoodPtr> localNeighborhoods;
        std::vector<CandidateGraspPtr> candidateGrasps;

        //objects necessary for testing candidate grasps:
        std::string eefName;
        //GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;
        VirtualRobot::RobotPtr robot;

    };
}

#endif /* __MAT_GRASP_PLANNER_H__ */
