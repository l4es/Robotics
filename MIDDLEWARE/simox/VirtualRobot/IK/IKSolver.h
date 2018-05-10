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
#ifndef _VirtualRobot_IKSolver_h_
#define _VirtualRobot_IKSolver_h_

#include "../VirtualRobotImportExport.h"

#include "../Nodes/RobotNode.h"
#include "../RobotNodeSet.h"

#include <string>
#include <vector>



namespace VirtualRobot
{


    class VIRTUAL_ROBOT_IMPORT_EXPORT IKSolver : public boost::enable_shared_from_this<IKSolver>
    {
    public:

        /*!
            @brief Flags for the selection of the target components.
            @details The flags can be combined with the +-operator.
        */
        enum CartesianSelection
        {
            X = 1,
            Y = 2,
            Z = 4,
            Position = 7,
            Orientation = 8,
            All = 15
        };

        /*!
            @brief Initialize a IK solver without collision detection.
            \param rns The robotNodes (i.e., joints) for which the Jacobians should be calculated.
        */
        IKSolver(RobotNodeSetPtr rns);


        /*!
            Setup collision detection
            \param avoidCollisionsWith The IK solver will consider collision checks between rns and avoidCollisionsWith
        */
        virtual void collisionDetection(SceneObjectPtr avoidCollisionsWith);
        /*!
            Setup collision detection
            \param avoidCollisionsWith The IK solver will consider collision checks between rns and avoidCollisionsWith
        */
        virtual void collisionDetection(ObstaclePtr avoidCollisionsWith);
        /*!
            Setup collision detection
            \param avoidCollisionsWith The IK solver will consider collision checks between rns and avoidCollisionsWith
        */
        virtual void collisionDetection(SceneObjectSetPtr avoidCollisionsWith);
        /*!
            Setup collision detection
            \param avoidCollision The IK solver will consider collision checks, defined in this CDManager instance.
        */
        virtual void collisionDetection(CDManagerPtr avoidCollision);



        /*!
            Here, the default values of the maximum allowed error in translation and orientation can be changed.
            \param maxErrorPositionMM The maximum position error that is allowed, given in millimeter.
            \param maxErrorOrientationRad The maximum orientation error that is allowed, given in radian (0.02 is approx 1 degree).
        */
        virtual void setMaximumError(float maxErrorPositionMM = 1.0f, float maxErrorOrientationRad = 0.02);

        /*!
            When set, the reachability data is used to quickly decide if a given pose or grasp is reachable or not.
            This option can be enabled by setting the reachability space and it can be disabled by setting an empty ReachabilityPtr.
        */
        virtual void setReachabilityCheck(ReachabilityPtr reachabilitySpace);

        /*!
            This method solves the IK up to the specified max error. On success, the joints of the the corresponding RobotNodeSet are set to the IK solution.
            \param globalPose The target pose given in global coordinate system.
            \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
            \param maxLoops An optional parameter, if multiple tries should be made
            \return true on success
        */
        virtual bool solve(const Eigen::Matrix4f& globalPose, CartesianSelection selection = All, int maxLoops = 1) = 0;



        /*!
            This method solves the IK up to the specified max error. The joints of the robot node set are not updated.
            \param globalPose The target pose given in global coordinate system.
            \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
            \return The joint angles are returned as std::vector
        */
        virtual std::vector<float> solveNoRNSUpdate(const Eigen::Matrix4f& globalPose, CartesianSelection selection = All);

        /*!
            Convenient method to solve IK queries without considering orientations.
        */
        virtual bool solve(const Eigen::Vector3f& globalPosition);

        /*!
            This method solves the IK up to the specified max error. On success, the joints of the the corresponding RobotNodeSet are set to the IK solution.
            \param object The object.
            \param selection Select the parts of the global pose that should be used for IK solving. (e.g. you can just consider the position and ignore the target orientation)
            \param maxLoops How many tries.
            \return On success: The grasp for which an IK-solution was found, otherwise an empty GraspPtr
        */
        virtual GraspPtr solve(ManipulationObjectPtr object, CartesianSelection selection = All, int maxLoops = 1);
        virtual bool solve(ManipulationObjectPtr object, GraspPtr grasp, CartesianSelection selection = All, int maxLoops = 1);


        /*!
            Try to find a solution for grasping the object with the given GraspSet.
            \return On success: The grasp for which an IK-solution was found, otherwise an empty GraspPtr
        */
        virtual GraspPtr sampleSolution(ManipulationObjectPtr object, GraspSetPtr graspSet, CartesianSelection selection = All, bool removeGraspFromSet = false, int maxLoops = 1);

        /*!
            This method returns true, when no reachability data is specified for this IK solver.
            If there is an reachability space defined, it is queried weather the pose is within the reachability or not and the result is returned.
        */
        virtual bool checkReachable(const Eigen::Matrix4f& globalPose);


        RobotNodeSetPtr getRobotNodeSet();
        RobotNodePtr getTcp();

        void setVerbose(bool enable);

    protected:


        //! This method should deliver solution sample out of the set of possible solutions
        virtual bool _sampleSolution(const Eigen::Matrix4f& globalPose, CartesianSelection selection, int maxLoops = 1)
        {
            return solve(globalPose, selection, maxLoops);
        }

        RobotNodeSetPtr rns;
        RobotNodePtr tcp;

        CDManagerPtr cdm;

        float maxErrorPositionMM;
        float maxErrorOrientationRad;

        ReachabilityPtr reachabilitySpace;

        bool verbose;
    };

    typedef boost::shared_ptr<IKSolver> IKSolverPtr;
} // namespace VirtualRobot

#endif // _VirtualRobot_IKSolver_h_
