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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_GenericIKSolver_h_
#define _VirtualRobot_GenericIKSolver_h_

#include "../VirtualRobotImportExport.h"
#include "IKSolver.h"
#include "DifferentialIK.h"
#include "../ManipulationObject.h"

namespace VirtualRobot
{


    class VIRTUAL_ROBOT_IMPORT_EXPORT GenericIKSolver : public IKSolver
    {
    public:

        /*!
            @brief Initialize an IK solver without collision detection.
            \param rns The robotNodes (i.e., joints) for which the Jacobians should be calculated.
            \param invJacMethod The method that should be used to compute the inverse of the Jacobian.
        */
        GenericIKSolver(RobotNodeSetPtr rns, JacobiProvider::InverseJacobiMethod invJacMethod = JacobiProvider::eSVD);

        /*!
            This method solves the IK up to the specified max error. On success, the joints of the the corresponding RobotNodeSet are set to the IK solution.
            \param globalPose The target pose given in global coordinate system.
            \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
            \param maxLoops How often should we try.
            \return true on success
        */
        virtual bool solve(const Eigen::Matrix4f& globalPose, CartesianSelection selection = All, int maxLoops = 1);

        /*!
            This method solves the IK up to the specified max error. On success, the joints of the the corresponding RobotNodeSet are set to the IK solution.
            \param object The grasps of this object are checked if the stored TCP is identical with teh TCP of teh current RobotNodeSet, and the an IK solution for one of remaining grasps is searched.
            \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
            \param maxLoops How often should we try.
            \return On success: The grasp for which an IK-solution was found, otherwise an empty GraspPtr
        */
        virtual GraspPtr solve(ManipulationObjectPtr object, CartesianSelection selection = All, int maxLoops = 1);
        virtual bool solve(ManipulationObjectPtr object, GraspPtr grasp, CartesianSelection selection = All, int maxLoops = 1);

        void setupJacobian(float stepSize, int maxLoops);

        void setVerbose(bool enable);

        DifferentialIKPtr getDifferentialIK();

        void setupTranslationalJoint(RobotNodePtr rn, float initialValue);


    protected:

        //! This method is called by the constructor and can be used in derived classes for initialization.
        virtual void _init();

        virtual bool _sampleSolution(const Eigen::Matrix4f& globalPose, CartesianSelection selection, int maxLoops = 1);

        RobotNodePtr coordSystem;
        JacobiProvider::InverseJacobiMethod invJacMethod;
        bool trySolve();
        void setJointsRandom();

        DifferentialIKPtr jacobian;
        float jacobianStepSize;
        int jacobianMaxLoops;

        RobotNodePtr translationalJoint;
        float initialTranslationalJointValue;
    };

    typedef boost::shared_ptr<GenericIKSolver> GenericIKSolverPtr;
} // namespace VirtualRobot

#endif // _VirtualRobot_GenericIKSolver_h_
