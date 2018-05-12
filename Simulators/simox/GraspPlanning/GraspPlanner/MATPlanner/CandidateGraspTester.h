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
#ifndef CANDIDATEGRASPTESTER_H
#define CANDIDATEGRASPTESTER_H

#include "../../GraspStudio.h"
#include "CandidateGrasp.h"
#include <vector>
#include "MedialSphere.h"

#include "../../ExternalDependencies/powercrust/powercrust.h"

namespace GraspStudio
{
    class CandidateGraspTester;
    typedef boost::shared_ptr<CandidateGraspTester> CandidateGraspTesterPtr;


    class GRASPSTUDIO_IMPORT_EXPORT CandidateGraspTester
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CandidateGraspTester(VirtualRobot::RobotPtr robot, VirtualRobot::SceneObjectPtr object,
                             std::string eefName, GraspQualityMeasureWrenchSpacePtr qualityMeasure, bool verbose);

        void testCandidate(CandidateGraspPtr candidate);

    protected:
        void closeEEFAndTest(CandidateGraspPtr candidate, float positionScaleFactor);
        void openEEF();
        void setEEF(Eigen::Matrix4f& poseGCP);
        void moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops);
        bool updateEEFPose(const Eigen::Vector3f& deltaPosition);
        bool updateEEFPose(const Eigen::Matrix4f& deltaPose);

        VirtualRobot::RobotPtr robot;
        VirtualRobot::SceneObjectPtr object;
        VirtualRobot::EndEffectorPtr eef;
        VirtualRobot::RobotPtr eefRobot;
        VirtualRobot::EndEffectorPtr eefCloned;
        std::string eefName;
        GraspQualityMeasureWrenchSpacePtr qualityMeasure;
        bool verbose;
    };
}

#endif // CANDIDATEGRASPTESTER_H
