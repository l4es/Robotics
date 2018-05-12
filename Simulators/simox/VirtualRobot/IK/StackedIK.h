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
#ifndef _VirtualRobot_StackedIK_h_
#define _VirtualRobot_StackedIK_h_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/JacobiProvider.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/VirtualRobotImportExport.h>
#include <VirtualRobot/IK/HierarchicalIK.h>


namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT StackedIK : public boost::enable_shared_from_this<StackedIK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        StackedIK(RobotNodeSetPtr rns, JacobiProvider::InverseJacobiMethod method = JacobiProvider::eSVD);

        virtual ~StackedIK();

        Eigen::VectorXf computeStep(const std::vector<JacobiProviderPtr> &jacDefs, float stepSize = 0.2f);

        void setVerbose(bool v);

    protected:
        RobotNodeSetPtr rns;
        JacobiProvider::InverseJacobiMethod method;
        bool verbose;
    };

    typedef boost::shared_ptr<StackedIK> StackedIKPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_StackedIK_h_


