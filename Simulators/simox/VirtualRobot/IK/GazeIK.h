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
* @copyright  2014 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_GazeIK_h_
#define _VirtualRobot_GazeIK_h_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/IK/JointLimitAvoidanceJacobi.h>
#include <VirtualRobot/IK/HierarchicalIK.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <VirtualRobot/VirtualRobotImportExport.h>


namespace VirtualRobot
{

    /*!
        A specialized IK solver for gaze related tasks. The gaze is represented by a virtual translational joint and
        the IK problem is solved by searching a solution for the head joints and the virtual gaze joint so that the position
        of the end point is at the requested gaze position.
        By default joint limit avoidance is considered as secondary task in order to generate more naturally looking configurations.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT GazeIK : public boost::enable_shared_from_this<GazeIK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            \param rns The RobotNodeSet containing the neck (optionally a common eye tilt joint) and the virtual translational joint that represents the gaze.
                       The TCP of the RNS is tried to position at the goal pose.
            \param virtualTranslationJoint A prismatic joint (usually without any models and physics properties) that represents the gaze
        */
        GazeIK(RobotNodeSetPtr rns, RobotNodePrismaticPtr virtualTranslationJoint);

        /*!
            The standard setup can be changed with this method.
            \param maxPosError Specifies how exact the target should be reached (standard 5mm).
            \param maxLoops Specifies how many seeds should be generate for starting a gradient descent search (standard 30)
            \param maxGradientDecentSteps The maximum number of steps that should be taken to decent the gradient
        */
        void setup(float maxPosError, int maxLoops, int maxGradientDecentSteps);

        /*!
            Joint Limit Avoidance is considered as secondary task.
            \param enable The joint limit avoidance can be enabled (standard) or disabled.
        */
        void enableJointLimitAvoidance(bool enable);

        /*!
            Compute a gradient descent step.
            \param goal The goal position of the gaze in global coordinate system.
            \param stepSize Can be used to limit the error step.
            \return the joint delta vector
        */
        Eigen::VectorXf computeStep(const Eigen::Vector3f &goal, float stepSize = 0.6f);

        bool solve(const Eigen::Vector3f &goal, float stepSize = 0.6f);

        float getMaxPosError();

        void setVerbose(bool v);

    protected:

        void setupIK();
        void setJointsRandom();
        void setJointsRandom(const Eigen::Vector3f &goal, int bestOfTries); // do random joint config bestOfTries times and determine the best in terms of tcp to goal distance (good guess)
        bool trySolve(const Eigen::Vector3f &goal, float stepSize);

        float getCurrentError(const Eigen::Vector3f &goal);
        void applyJLA(const Eigen::Vector3f &goal, int steps, float stepSize);

        bool checkTolerances(const Eigen::Vector3f &goal);

        std::vector<RobotNodePtr> nodes;
        RobotNodeSetPtr rns;
        RobotNodePrismaticPtr virtualTranslationJoint;

        HierarchicalIKPtr ikSolver;
        DifferentialIKPtr ikGaze;
        JointLimitAvoidanceJacobiPtr ikJointLimits;

        bool enableJLA;
        int maxLoops;
        int maxGradientDecentSteps;
        float maxPosError;
        int randomTriesToGetBestConfig;
        bool verbose;
    };

    typedef boost::shared_ptr<GazeIK> GazeIKPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_GazeIK_h_

