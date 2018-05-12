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
#include "CandidateGrasp.h"
#include "StrOutHelpers.h"

using namespace std;
using namespace Eigen;

namespace GraspStudio
{


    CandidateGrasp::CandidateGrasp()
    {
        graspTargetPoint.setZero();
        handApproachDirection.setZero();
        handOrientation.setZero();
        handPreshape.setZero();
        candidateGraspType = "None";

        tested = false;
        finalHandPose.setZero();
        quality = -1.0;

    }

    void CandidateGrasp::printDebug()
    {
        cout << "=== CandidateGrasp: ===" << std::endl;
        cout << "graspTargetPoint: " << StrOutHelpers::toString(graspTargetPoint) << endl;
        cout << "handApproachDirection: " << StrOutHelpers::toString(handApproachDirection) << endl;
        cout << "handOrientation: " << StrOutHelpers::toString(handOrientation) << endl;
        //cout << "handPreshape: " << StrOutHelpers::toString(handPreshape) << endl;
        cout << "candidateGraspType: " << candidateGraspType.c_str() << endl;
        cout << "tested: " << tested << endl;

        if (tested)
        {
            //        cout << "finalHandPose: " << finalHandPose << endl;
            //cout << "finalJointAngles: " << StrOutHelpers::toString(finalJointAngles) << endl;
            //        cout << "qualityVolume: " << qualityVolume << endl;
            cout << "quality: " << quality << endl;
        }

    }

    Matrix4f CandidateGrasp::toMatrix4f(float positionScaleFactor)
    {
        //approach dir now has to point TOWARDS the object (not away from it)
        Eigen::Vector3f tempHandApproachDir = -1 * handApproachDirection;

        //hand orientation: new y axis
        //approach direction: new z axis
        //new x axis: cross(y,z)

        Matrix4f cgAsMatrix;
        cgAsMatrix.setZero();
        cgAsMatrix.block<3, 1>(0, 1) = handOrientation;
        cgAsMatrix.block<3, 1>(0, 2) = tempHandApproachDir;
        cgAsMatrix.block<3, 1>(0, 0) = handOrientation.cross(tempHandApproachDir);
        cgAsMatrix.block<3, 1>(0, 3) = positionScaleFactor * graspTargetPoint;
        cgAsMatrix(3, 3) = 1.0f;

        return cgAsMatrix;
    }
}
