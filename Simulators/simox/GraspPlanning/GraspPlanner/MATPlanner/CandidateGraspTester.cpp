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
#include "CandidateGraspTester.h"
#include "../../GraspQuality/GraspQualityMeasure.h"
#include "../../GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/EndEffector/EndEffector.h>

using namespace std;
using namespace Eigen;

namespace GraspStudio
{

    CandidateGraspTester::CandidateGraspTester(VirtualRobot::RobotPtr robot, VirtualRobot::SceneObjectPtr object,
            std::string eefName, GraspQualityMeasureWrenchSpacePtr qualityMeasure, bool verbose)
        : robot(robot), object(object), eefName(eefName), qualityMeasure(qualityMeasure), verbose(verbose)
    {
        if (!robot)
        {
            VR_ERROR << "no robot" << endl;
            return;
        }

        eef = robot->getEndEffector(eefName);

        if (!eef)
        {
            VR_ERROR << "FAILED to get eef with name " << eefName << endl;
            return;
        }

        eefRobot = eef->createEefRobot(eef->getName(), eef->getName());
        eefCloned = eefRobot->getEndEffector(eef->getName());
    }

    void CandidateGraspTester::setEEF(Eigen::Matrix4f& poseGCP)
    {
        if (!eefRobot || !eefCloned)
        {
            return;
        }

        eefRobot->setGlobalPoseForRobotNode(eefCloned->getGCP(), poseGCP);
    }

    void CandidateGraspTester::testCandidate(CandidateGraspPtr candidate)
    {
        //set hand (eef) to start pose:
        float positionScaleFactor = 1000.0;     //ACHTUNG: als Argument...!
        Eigen::Matrix4f poseGCP = candidate->toMatrix4f(positionScaleFactor);
        setEEF(poseGCP);

        openEEF();
        closeEEFAndTest(candidate, positionScaleFactor);
    }

    void CandidateGraspTester::moveEEFAway(const Eigen::Vector3f& approachDir, float step, int maxLoops)
    {
        VirtualRobot::SceneObjectSetPtr sos = eefCloned->createSceneObjectSet();

        if (!sos)
        {
            return;
        }

        int loop = 0;
        Eigen::Vector3f delta = approachDir * step;

        while (loop < maxLoops && eefCloned->getCollisionChecker()->checkCollision(object->getCollisionModel(), sos))
        {
            updateEEFPose(delta);
            loop++;
        }
    }

    bool CandidateGraspTester::updateEEFPose(const Eigen::Vector3f& deltaPosition)
    {
        Eigen::Matrix4f deltaPose;
        deltaPose.setIdentity();
        deltaPose.block(0, 3, 3, 1) = deltaPosition;
        return updateEEFPose(deltaPose);
    }

    bool CandidateGraspTester::updateEEFPose(const Eigen::Matrix4f& deltaPose)
    {
        Eigen::Matrix4f pose = eefCloned->getGCP()->getGlobalPose();
        pose = deltaPose * pose;
        eefRobot->setGlobalPoseForRobotNode(eefCloned->getGCP(), pose);
        return true;
    }

    void CandidateGraspTester::closeEEFAndTest(CandidateGraspPtr candidate, float positionScaleFactor)
    {
        if (!eefCloned || !qualityMeasure)
        {
            return;
        }

        //cout << "todo: moveaway..." << endl;
        moveEEFAway(candidate->handApproachDirection, 3.0f, 5); //MP 2013-10-01

        candidate->contacts.clear();

        if (eefCloned && eefRobot)
        {
            candidate->contacts = eefCloned->closeActors(object);
            qualityMeasure->setContactPoints(candidate->contacts);

            candidate->quality = qualityMeasure->getGraspQuality();
            candidate->forceClosure = qualityMeasure->isGraspForceClosure();

            if (verbose)
            {
                std::stringstream ss;
                ss << std::setprecision(3);
                ss << "Quality (wrench space): "
                   << candidate->quality << "\nForce closure: ";

                if (candidate->forceClosure)
                {
                    ss << "yes";
                }
                else
                {
                    ss << "no";
                }

                cout << ss.str();
            }

        }

        candidate->tested = true;

        candidate->finalHandPose = eefCloned->getGCP()->getGlobalPose();
        candidate->finalJointAngles = eefCloned->getConfiguration();
    }



    void CandidateGraspTester::openEEF()
    {
        if (!eefCloned)
        {
            return;
        }

        if (eefCloned && eefRobot)
        {
            eefCloned->openActors();
        }
    }


}
