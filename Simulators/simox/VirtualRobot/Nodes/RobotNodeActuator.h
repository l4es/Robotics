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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_RobotNodeActuator_h_
#define _VirtualRobot_RobotNodeActuator_h_

#include "../VirtualRobotImportExport.h"

#include "RobotNode.h"
#include <Eigen/Core>

namespace VirtualRobot
{
    /*!
        An interface definition for RobotNode actuators.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeActuator
    {
    public:
        /*!
            Constructor
        */
        RobotNodeActuator(RobotNodePtr node);               //!< The node to actuate

        /*!
        */
        virtual ~RobotNodeActuator();

        virtual void updateVisualizationPose(const Eigen::Matrix4f& pose, bool updateChildren = false);
        virtual void updateVisualizationPose(const Eigen::Matrix4f& pose, float jointValue, bool updateChildren = false);
        //! Just sets the joint angle without performing any calculations, model updates
        virtual void updateJointAngle(float jointValue);

        RobotNodePtr getRobotNode();
    protected:

        RobotNodePtr robotNode;
    };


    typedef boost::shared_ptr<RobotNodeActuator> RobotNodeActuatorPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_RobotNodeActuator_h_
