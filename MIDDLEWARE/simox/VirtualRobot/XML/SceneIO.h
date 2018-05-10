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
#ifndef _VirtualRobot_SceneIO_h_
#define _VirtualRobot_SceneIO_h_

#include "../VirtualRobotImportExport.h"
#include "BaseIO.h"
#include "../Scene.h"

// using forward declarations here, so that the rapidXML header does not have to be parsed when this file is included
namespace rapidxml
{
    template<class Ch>
    class xml_node;
};

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT SceneIO : public BaseIO
    {
    public:

        /*!
            Load scene from file.
            \param xmlFile The file.
            \return Returns an empty pointer, when file access failed.
        */
        static ScenePtr loadScene(const std::string& xmlFile);

        /*!
            Save a scene to file.
            \param s The scene to be saved.
            \param xmlFile The absolute filename.
            \return true on success.
        */
        static bool saveScene(ScenePtr s, const std::string& xmlFile);

        /*!
            Creates scene from string.
            \param xmlString The input.
            \param basePath If any robot tags are given, the base path for searching the robot files can be specified.
        */
        static ScenePtr createSceneFromString(const std::string& xmlString, const std::string& basePath = "");

    protected:

        // instantiation not allowed
        SceneIO();
        virtual ~SceneIO();
        static ScenePtr processScene(rapidxml::xml_node<char>* sceneXMLNode, const std::string& basePath);
        static ScenePtr processSceneAttributes(rapidxml::xml_node<char>* sceneXMLNode);
        static bool processSceneRobot(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene, const std::string& basePath);
        static bool processSceneObstacle(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene, const std::string& basePath);
        static bool processSceneTrajectory(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene);
        static bool processSceneManipulationObject(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene, const std::string& basePath);
        static bool processSceneObjectSet(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene);
    };

}

#endif // _VirtualRobot_SceneIO_h_
