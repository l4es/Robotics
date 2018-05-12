
#include "SceneIO.h"
#include "../VirtualRobotException.h"
#include "rapidxml.hpp"


#include "RobotIO.h"
#include "ObjectIO.h"
#include "../Trajectory.h"
#include "../SceneObjectSet.h"

namespace VirtualRobot
{


    SceneIO::SceneIO()
    {
    }

    SceneIO::~SceneIO()
    {
    }

    VirtualRobot::ScenePtr SceneIO::loadScene(const std::string& xmlFile)
    {
        // load file
        std::ifstream in(xmlFile.c_str());

        THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << xmlFile);

        std::stringstream buffer;
        buffer << in.rdbuf();
        std::string robotXML(buffer.str());
        boost::filesystem::path filenameBaseComplete(xmlFile);
        boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
        std::string basePath = filenameBasePath.string();

        in.close();

        VirtualRobot::ScenePtr res = createSceneFromString(robotXML, basePath);
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file " << xmlFile);

        return res;
    }

    ScenePtr SceneIO::processSceneAttributes(rapidxml::xml_node<char>* sceneXMLNode)
    {
        // process attributes of scene

        // get name
        std::string sceneName = processNameAttribute(sceneXMLNode);
        THROW_VR_EXCEPTION_IF(sceneName.empty(), "Scene definition expects attribute 'name'");

        // build scene
        ScenePtr scene(new Scene(sceneName));
        return scene;
    }


    bool SceneIO::processSceneRobot(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!sceneXMLNode, "NULL data in processSceneRobot");

        // get name
        std::string robotName = processNameAttribute(sceneXMLNode, true);

        if (robotName.empty())
        {
            THROW_VR_EXCEPTION("Please specify the name of the robot...");
            return false;
        }

        std::string initStr("initconfig");
        std::string initConfigName = processStringAttribute(initStr, sceneXMLNode, true);

        std::vector< RobotConfigPtr > configs;
        std::vector< std::vector< RobotConfig::Configuration > > configDefinitions;
        std::vector< std::string > configNames;
        Eigen::Matrix4f globalPose = Eigen::Matrix4f::Identity();
        std::string fileName;
        rapidxml::xml_node<>* node = sceneXMLNode->first_node();

        std::vector< rapidxml::xml_node<>* > rnsNodes;

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "file")
            {
                THROW_VR_EXCEPTION_IF(!fileName.empty(), "Multiple files defined in scene's robot tag '" << robotName << "'." << endl);
                fileName = processFileNode(node, basePath);
            }
            else if (nodeName == "configuration")
            {
                bool cOK = processConfigurationNodeList(node, configDefinitions, configNames);
                THROW_VR_EXCEPTION_IF(!cOK, "Invalid configuration defined in scene's robot tag '" << robotName << "'." << endl);
            }
            else if (nodeName == "globalpose")
            {
                processTransformNode(node, robotName, globalPose);
            }
            else if (nodeName == "robotnodeset")
            {
                rnsNodes.push_back(node);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in scene's Robot definition <" << robotName << ">." << endl);
            }

            node = node->next_sibling();
        }

        // create & register robot
        THROW_VR_EXCEPTION_IF(fileName.empty(), "Missing file definition in scene's robot tag '" << robotName << "'." << endl);
        RobotPtr robot = RobotIO::loadRobot(fileName);
        THROW_VR_EXCEPTION_IF(!robot, "Invalid robot file in scene's robot tag '" << robotName << "'." << endl);
        robot->setGlobalPose(globalPose);
        scene->registerRobot(robot);

        // create & register node sets
        int rnsNr = 0;

        for (size_t i = 0; i < rnsNodes.size(); i++)
        {
            // registers rns to robot
            RobotNodeSetPtr r = processRobotNodeSet(rnsNodes[i], robot, robot->getRootNode()->getName(), rnsNr);
            THROW_VR_EXCEPTION_IF(!r, "Invalid RobotNodeSet definition " << endl);
        }

        // create & register configs
        THROW_VR_EXCEPTION_IF(configDefinitions.size() != configNames.size(), "Invalid RobotConfig definitions " << endl);

        for (size_t i = 0; i < configDefinitions.size(); i++)
        {
            RobotConfigPtr rc(new RobotConfig(robot, configNames[i], configDefinitions[i]));
            scene->registerRobotConfig(robot, rc);
        }

        // process init config
        if (!initConfigName.empty())
        {
            THROW_VR_EXCEPTION_IF(!scene->hasRobotConfig(robot, initConfigName), "Scene's robot tag '" << robotName << "' does not have the initConfig '" << initConfigName << "'." << endl);
            robot->setJointValues(scene->getRobotConfig(robot, initConfigName));
        }

        return true;
    }



    bool SceneIO::processSceneObjectSet(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene)
    {
        THROW_VR_EXCEPTION_IF(!sceneXMLNode, "NULL data in processSceneObjectSet");

        // get name
        std::string sosName = processNameAttribute(sceneXMLNode, true);

        if (sosName.empty())
        {
            THROW_VR_EXCEPTION("Please specify the name of the scene object set...");
            return false;
        }

        SceneObjectSetPtr sos(new SceneObjectSet(sosName));

        rapidxml::xml_node<>* node = sceneXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "sceneobject")
            {
                std::string oname = processNameAttribute(node);

                THROW_VR_EXCEPTION_IF(!(scene->hasObstacle(oname) || scene->hasManipulationObject(oname)), " Object with name '" << oname << "' not known in scene '" << scene->getName() << "'." << endl);

                if (scene->hasObstacle(oname))
                {
                    sos->addSceneObject(scene->getObstacle(oname));
                }
                else
                {
                    sos->addSceneObject(scene->getManipulationObject(oname));
                }
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in scene's SceneObjectSet definition <" << sosName << ">." << endl);
            }

            node = node->next_sibling();
        }

        scene->registerSceneObjectSet(sos);
        return true;
    }


    bool SceneIO::processSceneManipulationObject(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!sceneXMLNode, "NULL data in processSceneManipulationObject");

        ManipulationObjectPtr o = ObjectIO::processManipulationObject(sceneXMLNode, basePath);

        if (!o)
        {
            return false;
        }

        scene->registerManipulationObject(o);
        return true;
    }


    bool SceneIO::processSceneObstacle(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!sceneXMLNode, "NULL data in processSceneObstacle");

        ObstaclePtr o = ObjectIO::processObstacle(sceneXMLNode, basePath);

        if (!o)
        {
            return false;
        }

        scene->registerObstacle(o);
        return true;
    }

    bool SceneIO::processSceneTrajectory(rapidxml::xml_node<char>* sceneXMLNode, ScenePtr scene)
    {
        THROW_VR_EXCEPTION_IF(!sceneXMLNode || !scene, "NULL data in processSceneTrajectory");

        std::vector<RobotPtr> robs = scene->getRobots();
        TrajectoryPtr o = BaseIO::processTrajectory(sceneXMLNode, robs);

        if (!o)
        {
            return false;
        }

        scene->registerTrajectory(o);
        return true;
    }

    ScenePtr SceneIO::processScene(rapidxml::xml_node<char>* sceneXMLNode, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!sceneXMLNode, "No <Scene> tag in XML definition");

        // process Attributes
        ScenePtr scene;
        scene = processSceneAttributes(sceneXMLNode);

        // process xml nodes
        std::vector<rapidxml::xml_node<char>* > sceneSetNodes;
        std::vector<rapidxml::xml_node<char>* > trajectoryNodes;


        rapidxml::xml_node<>* XMLNode = sceneXMLNode->first_node(NULL, 0, false);

        while (XMLNode)
        {
            std::string nodeName_ = XMLNode->name();
            std::string nodeName = getLowerCase(XMLNode->name());

            if (nodeName == "robot")
            {
                bool r = processSceneRobot(XMLNode, scene, basePath);

                if (!r)
                {
                    std::string failedNodeName = processNameAttribute(XMLNode);
                    THROW_VR_EXCEPTION("Failed to create robot " << failedNodeName << " in scene " << scene->getName() << endl);
                }

            }
            else if (nodeName == "obstacle")
            {
                bool r = processSceneObstacle(XMLNode, scene, basePath);

                if (!r)
                {
                    std::string failedNodeName = processNameAttribute(XMLNode);
                    THROW_VR_EXCEPTION("Failed to create obstacle " << failedNodeName << " in scene " << scene->getName() << endl);
                }
            }
            else if (nodeName == "manipulationobject")
            {
                bool r = processSceneManipulationObject(XMLNode, scene, basePath);

                if (!r)
                {
                    std::string failedNodeName = processNameAttribute(XMLNode);
                    THROW_VR_EXCEPTION("Failed to create ManipulationObject " << failedNodeName << " in scene " << scene->getName() << endl);
                }
            }
            else if (nodeName == "trajectory")
            {
                trajectoryNodes.push_back(XMLNode);
            }
            else if (nodeName == "sceneobjectset")
            {
                sceneSetNodes.push_back(XMLNode);

            }
            else
            {
                THROW_VR_EXCEPTION("XML node of type <" << nodeName_ << "> is not supported. Ignoring contents..." << endl);
            }

            XMLNode = XMLNode->next_sibling(NULL, 0, false);
        }

        // process all sceneSetNodes
        for (size_t i = 0; i < sceneSetNodes.size(); i++)
        {
            bool r = processSceneObjectSet(sceneSetNodes[i], scene);

            if (!r)
            {
                std::string failedNodeName = processNameAttribute(XMLNode);
                THROW_VR_EXCEPTION("Failed to create SceneObjectSet " << failedNodeName << " in scene " << scene->getName() << endl);
            }
        }


        // process all trajectories
        for (size_t i = 0; i < trajectoryNodes.size(); i++)
        {
            bool r = processSceneTrajectory(trajectoryNodes[i], scene);

            if (!r)
            {
                std::string failedNodeName = processNameAttribute(XMLNode);
                THROW_VR_EXCEPTION("Failed to create trajectory " << failedNodeName << " in scene " << scene->getName() << endl);
            }
        }

        return scene;
    }



    VirtualRobot::ScenePtr SceneIO::createSceneFromString(const std::string& xmlString, const std::string& basePath /*= ""*/)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::ScenePtr scene;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags
            rapidxml::xml_node<char>* sceneXMLNode = doc.first_node("Scene");
            scene = processScene(sceneXMLNode, basePath);
        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse data in xml definition" << endl
                               << "Error message:" << e.what() << endl
                               << "Position: " << endl << e.where<char>() << endl);
            return ScenePtr();
        }
        catch (VirtualRobot::VirtualRobotException&)
        {
            // rethrow the current exception
            delete[] y;
            throw;
        }
        catch (std::exception& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl
                               << "Error code:" << e.what() << endl);
            return ScenePtr();
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl);
            return ScenePtr();
        }

        delete[] y;
        return scene;
    }

    bool SceneIO::saveScene(ScenePtr s, const std::string& xmlFile)
    {
        if (!s)
        {
            VR_ERROR << "NULL data..." << endl;
            return false;
        }

        boost::filesystem::path filenameBaseComplete(xmlFile);
        filenameBaseComplete = boost::filesystem::system_complete(filenameBaseComplete);
        boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
        std::string basePath = filenameBasePath.string();

        std::string xmlString = s->getXMLString(basePath);

        // save file
        return BaseIO::writeXMLFile(xmlFile, xmlString, true);
    }


} // namespace VirtualRobot
