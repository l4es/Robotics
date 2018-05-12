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
#ifndef CANDIDATEGRASP_H
#define CANDIDATEGRASP_H

#include <VirtualRobot/EndEffector/EndEffector.h>
#include "../../GraspStudio.h"
#include <Eigen/Geometry>
#include "MedialSphere.h"
#include "LocalNeighborhood.h"

namespace GraspStudio
{

    class CandidateGrasp;
    typedef boost::shared_ptr<CandidateGrasp> CandidateGraspPtr;

    class GRASPSTUDIO_IMPORT_EXPORT CandidateGrasp
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        CandidateGrasp();
        bool tested;
        bool forceClosure;
        float quality;
        VirtualRobot::EndEffector::ContactInfoVector contacts;
        Eigen::Vector3f graspTargetPoint;
        Eigen::Vector3f handApproachDirection;
        Eigen::Vector3f handOrientation;
        Eigen::VectorXf handPreshape;

        std::string candidateGraspType;
        MedialSpherePtr medialSphere;
        LocalNeighborhoodPtr localNeighborhood;



        Eigen::Matrix4f finalHandPose;
        VirtualRobot::RobotConfigPtr finalJointAngles;


        void printDebug();

        Eigen::Matrix4f toMatrix4f(float positionScaleFactor = 1.0);
    };

}
#endif // CANDIDATEGRASP_H
