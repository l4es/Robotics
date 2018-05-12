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
#ifndef _VirtualRobot_HierarchicalIK_h_
#define _VirtualRobot_HierarchicalIK_h_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/JacobiProvider.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/VirtualRobotImportExport.h>


namespace VirtualRobot
{

    /*!
        With hierarchical IK methods several tasks/constraints can be considered for IK solving.
        Internally a hierarchical gradient descent is generated where the Nullspace of the preceding task definition
        is used for the computation of the joint delta in the current task.

        This implementation is based on the following publication:
        "A general framework for managing multiple tasks in highly redundant robotic systems.", Siciliano, B. ; Slotine, J.-J.E.,
        Advanced Robotics, 1991. 'Robots in Unstructured Environments', 91 ICAR., Fifth International Conference on

    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT HierarchicalIK : public boost::enable_shared_from_this<HierarchicalIK>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        HierarchicalIK(RobotNodeSetPtr rns);

        virtual ~HierarchicalIK();

        /*!
            computes hierarchical Jacobi step
            \param jacDefs All Jacobians an the corresponding error vectors that should be considered for computing a gradient step.
                           The jacProviders specify the Jacobian and InverseJacobian functions. Jacobians must cover the same RobotNodeSet (i.e., the same number of DoF).
                           The deltas specify the error for each Jacobian (e.g. in workspace). deltas[i].rows() must be equal to jacobies[i].rows().
            \param stepSize The deltas can be reduced in order to avoid oscillating behavior.
        */
        Eigen::VectorXf computeStep(const std::vector<JacobiProviderPtr> &jacDefs, float stepSize = 0.2f);

        void setVerbose(bool v);
    protected:

        RobotNodeSetPtr rns;
        bool verbose;
    };

    typedef boost::shared_ptr<HierarchicalIK> HierarchicalIKPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_HierarchicalIK_h_

