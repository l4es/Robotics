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
#ifndef _VirtualRobot_RobotNodeFixed_h_
#define _VirtualRobot_RobotNodeFixed_h_

#include "../VirtualRobotImportExport.h"

#include "RobotNode.h"
#include "../RobotFactory.h"

#include <Eigen/Core>

#include <string>
#include <vector>


namespace VirtualRobot
{
    class Robot;

    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotNodeFixed : public RobotNode
    {
    public:
        friend class RobotFactory;

        /*!
        Constructor
        */
        RobotNodeFixed(RobotWeakPtr rob,                //!< The robot
                       const std::string& name,                    //!< The name
                       const Eigen::Matrix4f& preJointTransform,   //!<  This is the fixed transformation of this RobotNode (used to compute globalPose)
                       VisualizationNodePtr visualization = VisualizationNodePtr(),//!< A visualization model
                       CollisionModelPtr collisionModel = CollisionModelPtr(),     //!< A collision model
                       const SceneObject::Physics& p = SceneObject::Physics(),     //!< physics information
                       CollisionCheckerPtr colChecker = CollisionCheckerPtr(),     //!< A collision checker instance (if not set, the global col checker is used)
                       RobotNodeType type = Generic);
        /*!
            Initialize with DH parameters.

            The DH parameters are all applied before! any visualization is added to the kinematic structure.
        */
        RobotNodeFixed(RobotWeakPtr rob,                        //!< The robot
                       const std::string& name,                            //!< The name
                       float a,                                            //!< Use fixed DH parameters to specify the transformation of this RobotNode
                       float d,                                            //!< Use fixed DH parameters to specify the transformation of this RobotNode
                       float alpha,                                        //!< Use fixed DH parameters to specify the transformation of this RobotNode
                       float theta,                                        //!< Use fixed DH parameters to specify the transformation of this RobotNode
                       VisualizationNodePtr visualization = VisualizationNodePtr(),  //!< A visualization model
                       CollisionModelPtr collisionModel = CollisionModelPtr(), //!< A collision model
                       const SceneObject::Physics& p = SceneObject::Physics(), //!< physics information
                       CollisionCheckerPtr colChecker = CollisionCheckerPtr(), //!< A collision checker instance (if not set, the global col checker is used)
                       RobotNodeType type = Generic);

        /*!
        */
        virtual ~RobotNodeFixed();

        virtual bool initialize(SceneObjectPtr parent = SceneObjectPtr(), const std::vector<SceneObjectPtr>& children = std::vector<SceneObjectPtr>());

        /*!
        Print status information.
        */
        virtual void print(bool printChildren = false, bool printDecoration = true) const;

    protected:
        //! Checks if nodeType constraints are fulfilled. Otherwise an exception is thrown. Called on initialization.
        virtual void checkValidRobotNodeType();

        RobotNodeFixed() {};
        virtual void updateTransformationMatrices(const Eigen::Matrix4f& parentPose);
        virtual RobotNodePtr _clone(const RobotPtr newRobot, const VisualizationNodePtr visualizationModel, const CollisionModelPtr collisionModel, CollisionCheckerPtr colChecker, float scaling);
        /*!
            Derived classes add custom XML tags here
        */
        virtual std::string _toXML(const std::string& modelPath);

    };

} // namespace VirtualRobot

#endif // _VirtualRobot_RobotNodeFixed_h_
