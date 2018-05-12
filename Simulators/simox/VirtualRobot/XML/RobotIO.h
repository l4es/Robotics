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
#ifndef _VirtualRobot_RobotIO_h_
#define _VirtualRobot_RobotIO_h_

#include "../VirtualRobotImportExport.h"
#include "../Units.h"
#include "../MathTools.h"
#include "../Robot.h"
#include "../Nodes/RobotNode.h"
#include "BaseIO.h"

#include <string>
#include <vector>
#include <map>
#include <fstream>



// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template<class Ch>
    class xml_node;
};

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT RobotIO : public BaseIO
    {
    public:

        enum RobotDescription
        {
            eFull,              // load complete robot definition
            eCollisionModel,    // skip visualization tags and load only collision model
            eStructure          // load only the structure of the robot, ignore visualization and collision tags -> faster access when robot is only used for coordinate transformations
        };

        /*!
                Loads robot from file.
                @param xmlFile The file
                @param loadMode Standard: eFull, When eStructure is used no visualization and collision models are loaded for faster access.
                @return Returns an empty pointer, when file access failed.
        */
        static RobotPtr loadRobot(const std::string& xmlFile, RobotDescription loadMode = eFull);

        /*!
                Creates Robot from string.
                @param xmlString The input string.
                @param basePath If any \<childFromRobot\> tags are given, the path for searching the robot files can be specified.
                @param loadMode Standard: eFull, When eStructure is used no visualization and collision models are loaded for faster access.
            */
        static RobotPtr createRobotFromString(const std::string& xmlString, const std::string& basePath = "", RobotDescription loadMode = eFull);


        /*!
            Creates an XML string that defines the robot and stores it to the file basePath/filename. All visualizations and collision models are stored to the basePath/modeDir directory
            @param robot The robot to save.
            @param filename The filename without path.
            @param basePath The directory to store the robot to
            @param modelDir The local directory where all visualization files should be stored to.
        */
        static bool saveXML(RobotPtr robot, const std::string& filename, const std::string& basePath, const std::string& modelDir = "models", bool storeEEF = true, bool storeRNS = true, bool storeSensors = true, bool storeModelFiles = true);


    protected:

        struct ChildFromRobotDef
        {
            std::string filename;
            bool importEEF;
        };

        // instantiation not allowed
        RobotIO();
        virtual ~RobotIO();

        static RobotPtr processRobot(rapidxml::xml_node<char>* robotXMLNode, const std::string& basePath, RobotDescription loadMode = eFull);
        static RobotPtr processRobotAttributes(rapidxml::xml_node<char>* robotXMLNode, std::string& robotRoot);
        static void processRobotChildNodes(rapidxml::xml_node<char>* robotXMLNode,
                                           RobotPtr robo,
                                           const std::string& robotRoot,
                                           const std::string& basePath,
                                           std::map< RobotNodePtr,
                                           std::vector<ChildFromRobotDef> >& childrenFromRobotFilesMap,
                                           std::vector<rapidxml::xml_node<char>* >& robotNodeSetNodes,
                                           std::vector<rapidxml::xml_node<char>* >& endeffectorNodes,
                                           RobotDescription loadMode = eFull);
        static RobotNodePtr processRobotNode(rapidxml::xml_node<char>* robotNodeXMLNode,
                                             RobotPtr robo,
                                             const std::string& basePath,
                                             int& robotNodeCounter,
                                             std::vector< std::string >& childrenNames,
                                             std::vector< ChildFromRobotDef >& childrenFromRobot,
                                             RobotDescription loadMode = eFull,
                                             RobotNode::RobotNodeType rntype = RobotNode::Generic);
        static EndEffectorPtr processEndeffectorNode(rapidxml::xml_node<char>* endeffectorXMLNode, RobotPtr robo);
        static EndEffectorActorPtr processEndeffectorActorNode(rapidxml::xml_node<char>* endeffectorActorXMLNode, RobotPtr robo);
        static void processEndeffectorStaticNode(rapidxml::xml_node<char>* endeffectorStaticXMLNode, RobotPtr robo, std::vector<RobotNodePtr>& staticNodesList);
        static EndEffectorActor::CollisionMode processEEFColAttributes(rapidxml::xml_node<char>* node, bool allowOtherAttributes = false);
        static void processActorNodeList(rapidxml::xml_node<char>* parentNode, RobotPtr robot, std::vector<EndEffectorActor::ActorDefinition>& actorList, bool clearList = true);
        //static RobotNodeSetPtr processRobotNodeSet(rapidxml::xml_node<char> *setXMLNode, RobotPtr robo, const std::string &rootName, int &robotNodeSetCounter);
        static void processChildNode(rapidxml::xml_node<char>* childXMLNode, std::vector<std::string>& childrenNames);
        static RobotNodePtr processJointNode(rapidxml::xml_node<char>* jointXMLNode, const std::string& robotNodeName,
                                             RobotPtr robot, VisualizationNodePtr visualizationNode, CollisionModelPtr collisionModel,
                                             SceneObject::Physics& physics, RobotNode::RobotNodeType rntype, Eigen::Matrix4f& transformationMatrix);
        static void processChildFromRobotNode(rapidxml::xml_node<char>* childXMLNode, const std::string& nodeName, std::vector< ChildFromRobotDef >& childrenFromRobot);
        static void processLimitsNode(rapidxml::xml_node<char>* limitsXMLNode, float& jointLimitLo, float& jointLimitHi);
        static bool processSensor(RobotNodePtr rn, rapidxml::xml_node<char>* sensorXMLNode, RobotDescription loadMode, const std::string& basePath);
        static std::map<std::string, int> robot_name_counter;
        static VisualizationNodePtr checkUseAsColModel(rapidxml::xml_node<char>* visuXMLNode, const std::string& robotNodeName, const std::string& basePath);
    };

}

#endif // _VirtualRobot_RobotIO_h_
