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
#ifndef _VirtualRobot_RobotFactory_h_
#define _VirtualRobot_RobotFactory_h_

#include "VirtualRobotImportExport.h"
#include "MathTools.h"

#include <string>
#include <vector>
#include <map>

namespace VirtualRobot
{

    class Robot;
    class RobotNode;

    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotFactory
    {
    public:
        /*!
        Creates an empty robot.
        */
        static RobotPtr createRobot(const std::string& name, const std::string& type = "");

        /*!
            Initializes Robot and all RobotNodes.
            \param robotNodes All nodes of the robot. Must contain rootNode.
            \param childrenMap Parent-child relations are built according to this data.
            \param rootNode The root.
        */
        static bool initializeRobot(RobotPtr robot,
                                    std::vector<RobotNodePtr >& robotNodes,
                                    std::map< RobotNodePtr, std::vector<std::string> > childrenMap,
                                    RobotNodePtr rootNode);


        struct robotNodeDef
        {
            std::string name;
            std::vector<std::string> children;
            // used to mark children whose transformation should be inverted
            std::vector<bool> invertTransformation;
        };

        struct robotStructureDef
        {
            std::string rootName;
            std::vector<robotNodeDef> parentChildMapping;
        };

        static RobotPtr cloneInversed(RobotPtr robot, const std::string& newRootName);

        static RobotPtr cloneChangeStructure(RobotPtr robot, robotStructureDef& newStructure);

        /*! Clone kinematic chain and reverse direction.
         *
         * \param startNode Name of the start node of the original kinematic chain.
         * \param endNode Name of the end node of the original kinematic chain. Will be the new root.
         */
        static RobotPtr cloneChangeStructure(RobotPtr robot, const std::string& startNode, const std::string& endNode);

        /*!
         * \brief attach Attach an object to a robot. The object is cloned.
         * \param robot
         * \param o The object and its visualization model is cloned
         * \param rn The robot node to which the object should be attached
         * \param transformation The RN to object transformation
         * \return true on succes
         */
        static bool attach(RobotPtr robot, SceneObjectPtr o, RobotNodePtr rn, const Eigen::Matrix4f & transformation);

        static bool detach(RobotPtr robot, RobotNodePtr rn);

    protected:
        // instantiation not allowed
        RobotFactory();
        virtual ~RobotFactory();

        //static bool initRobotNode(RobotNodePtr n, RobotNodePtr parent, std::vector< RobotNodePtr > &robotNodes);
    };

}

#endif // _VirtualRobot_RobotFactory_h_
