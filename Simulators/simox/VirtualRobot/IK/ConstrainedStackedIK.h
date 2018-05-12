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
#ifndef _VirtualRobot_ConstrainedStackedIK_h_
#define _VirtualRobot_ConstrainedStackedIK_h_

#include <VirtualRobot/VirtualRobotImportExport.h>
#include <VirtualRobot/IK/ConstrainedIK.h>
#include <VirtualRobot/IK/JacobiProvider.h>
#include <VirtualRobot/IK/StackedIK.h>

#include <boost/shared_ptr.hpp>

namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT ConstrainedStackedIK : public ConstrainedIK, public boost::enable_shared_from_this<ConstrainedStackedIK>
    {
        public:
            ConstrainedStackedIK(RobotPtr &robot, const RobotNodeSetPtr &nodeSet, float stepSize = 0.2f, int maxIterations = 1000,
                                 JacobiProvider::InverseJacobiMethod method = JacobiProvider::eSVD);

            bool initialize();

            virtual bool solveStep();

        protected:
            RobotNodeSetPtr nodeSet;
            StackedIKPtr ik;

            std::vector<JacobiProviderPtr> jacobians;

            JacobiProvider::InverseJacobiMethod method;

            float stepSize;
    };

    typedef boost::shared_ptr<ConstrainedStackedIK> ConstrainedStackedIKPtr;
}

#endif
