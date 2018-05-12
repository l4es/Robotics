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
* @author     Peter Kaiser
* @copyright  2015 Peter Kaiser
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_BalanceConstraint_h_
#define _VirtualRobot_BalanceConstraint_h_

#include <VirtualRobot/VirtualRobotImportExport.h>
#include <VirtualRobot/IK/Constraint.h>
#include <VirtualRobot/IK/CoMIK.h>
#include <VirtualRobot/IK/SupportPolygon.h>

#include <boost/shared_ptr.hpp>

class SoSeparator;
class SoNode;

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT BalanceConstraint : public Constraint, public boost::enable_shared_from_this<BalanceConstraint>
    {
        public:
            BalanceConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SceneObjectSetPtr &contactNodes,
                              float tolerance=0.1f, float minimumStability=0.5f, float maxSupportDistance=10.0f, bool supportPolygonUpdates=true, bool considerCoMHeight = false);
            BalanceConstraint(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SupportPolygonPtr &supportPolygon,
                              float tolerance=0.1f, float minimumStability=0.5f, float maxSupportDistance=10.0f, bool supportPolygonUpdates=true, bool considerCoMHeight = false);

            Eigen::MatrixXf getJacobianMatrix();
            Eigen::MatrixXf getJacobianMatrix(SceneObjectPtr tcp);
            Eigen::VectorXf getError(float stepSize = 1.0f);
            bool checkTolerances();

            bool getRobotPoseForConstraint(RobotPtr &robot, Eigen::Matrix4f &pose);
            Eigen::Vector3f getCoM();
            SupportPolygonPtr getSupportPolygon();

            std::string getConstraintType();
            void setCoMHeight(float height);
        protected:
            void initialize(const RobotPtr &robot, const RobotNodeSetPtr &joints, const RobotNodeSetPtr &bodies, const SceneObjectSetPtr &contactNodes,
                            float tolerance, float minimumStability, float maxSupportDistance, bool supportPolygonUpdates, bool considerCoMHeight);

            void updateSupportPolygon();

            void visualizeSupportPolygon(SoSeparator *sep);



        protected:
            CoMIKPtr comIK;
            SupportPolygonPtr supportPolygon;

            float height;
            bool considerCoMHeight;

            RobotNodeSetPtr joints;
            RobotNodeSetPtr bodies;

            float minimumStability;
            float maxSupportDistance;
            float tolerance;
            bool supportPolygonUpdates;
    };

    typedef boost::shared_ptr<BalanceConstraint> BalanceConstraintPtr;
}

#endif
