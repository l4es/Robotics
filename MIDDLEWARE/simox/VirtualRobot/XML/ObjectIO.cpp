
#include "ObjectIO.h"
#include "../VirtualRobotException.h"
#include "rapidxml.hpp"


#include "RobotIO.h"
#include "../Grasping/Grasp.h"
#include "../Grasping/GraspSet.h"
#include "../ManipulationObject.h"

namespace VirtualRobot
{



    ObjectIO::ObjectIO()
    {
    }

    ObjectIO::~ObjectIO()
    {
    }

    VirtualRobot::ObstaclePtr ObjectIO::loadObstacle(const std::string& xmlFile)
    {
        // load file
        std::ifstream in(xmlFile.c_str());

        THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << xmlFile);

        boost::filesystem::path filenameBaseComplete(xmlFile);
        boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
        std::string basePath = filenameBasePath.string();
        VirtualRobot::ObstaclePtr res = loadObstacle(in, basePath);
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file " << xmlFile);
        res->setFilename(xmlFile);
        return res;
    }

    VirtualRobot::ObstaclePtr ObjectIO::loadObstacle(const std::ifstream& xmlFile, const std::string& basePath /*= ""*/)
    {
        // load file
        THROW_VR_EXCEPTION_IF(!xmlFile.is_open(), "Could not open XML file");

        std::stringstream buffer;
        buffer << xmlFile.rdbuf();
        std::string objectXML(buffer.str());

        VirtualRobot::ObstaclePtr res = createObstacleFromString(objectXML, basePath);
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file.");
        return res;
    }


    VirtualRobot::ManipulationObjectPtr ObjectIO::loadManipulationObject(const std::string& xmlFile)
    {
        // load file
        std::ifstream in(xmlFile.c_str());

        THROW_VR_EXCEPTION_IF(!in.is_open(), "Could not open XML file:" << xmlFile);

        boost::filesystem::path filenameBaseComplete(xmlFile);
        boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
        std::string basePath = filenameBasePath.string();
        VirtualRobot::ManipulationObjectPtr res = loadManipulationObject(in, basePath);
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file " << xmlFile);
        res->setFilename(xmlFile);
        return res;
    }

    VirtualRobot::ManipulationObjectPtr ObjectIO::loadManipulationObject(const std::ifstream& xmlFile, const std::string& basePath /*= ""*/)
    {
        // load file
        THROW_VR_EXCEPTION_IF(!xmlFile.is_open(), "Could not open XML file");

        std::stringstream buffer;
        buffer << xmlFile.rdbuf();
        std::string objectXML(buffer.str());

        VirtualRobot::ManipulationObjectPtr res = createManipulationObjectFromString(objectXML, basePath);
        res->initialize();
        THROW_VR_EXCEPTION_IF(!res, "Error while parsing file.");
        return res;
    }

    GraspPtr ObjectIO::processGrasp(rapidxml::xml_node<char>* graspXMLNode, const std::string& robotType, const std::string& eef, const std::string& objName)
    {
        THROW_VR_EXCEPTION_IF(!graspXMLNode, "No <Grasp> tag ?!");
        // get name
        std::string name = processNameAttribute(graspXMLNode, true);
        std::string method = processStringAttribute("creation", graspXMLNode, true);
        float quality = processFloatAttribute(std::string("quality"), graspXMLNode, true);
        std::string preshapeName = processStringAttribute("preshape", graspXMLNode, true);
        Eigen::Matrix4f pose;
        pose.setIdentity();
        std::vector< RobotConfig::Configuration > configDefinitions;
        std::string configName;

        rapidxml::xml_node<>* node = graspXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "transform")
            {
                processTransformNode(graspXMLNode, name, pose);

            }
            else if (nodeName == "configuration")
            {
                THROW_VR_EXCEPTION_IF(configDefinitions.size() > 0, "Only one configuration per grasp allowed");
                bool cOK = processConfigurationNode(node, configDefinitions, configName);
                THROW_VR_EXCEPTION_IF(!cOK, "Invalid configuration defined in grasp tag '" << name << "'." << endl);

            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Grasp <" << name << ">." << endl);
            }

            node = node->next_sibling();
        }

        GraspPtr grasp(new Grasp(name, robotType, eef, pose, method, quality, preshapeName));

        if (configDefinitions.size() > 0)
        {
            // create & register configs
            std::map< std::string, float > rc;

            for (size_t i = 0; i < configDefinitions.size(); i++)
            {
                rc[ configDefinitions[i].name ] = configDefinitions[i].value;
            }

            grasp->setConfiguration(rc);
        }

        return grasp;
    }


    GraspSetPtr ObjectIO::processGraspSet(rapidxml::xml_node<char>* graspSetXMLNode, const std::string& objName)
    {
        THROW_VR_EXCEPTION_IF(!graspSetXMLNode, "No <GraspSet> tag ?!");

        // get name
        std::string gsName = processNameAttribute(graspSetXMLNode, true);
        std::string gsRobotType = processStringAttribute(std::string("robottype"), graspSetXMLNode, true);
        std::string gsEEF = processStringAttribute(std::string("endeffector"), graspSetXMLNode, true);

        if (gsName.empty() || gsRobotType.empty() || gsEEF.empty())
        {
            THROW_VR_EXCEPTION("GraspSet tags must have valid attributes 'Name', 'RobotType' and 'EndEffector'");
        }

        GraspSetPtr result(new GraspSet(gsName, gsRobotType, gsEEF));

        rapidxml::xml_node<>* node = graspSetXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "grasp")
            {
                GraspPtr grasp = processGrasp(node, gsRobotType, gsEEF, objName);
                THROW_VR_EXCEPTION_IF(!grasp, "Invalid 'Grasp' tag in '" << objName << "'." << endl);
                result->addGrasp(grasp);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in GraspSet <" << gsName << ">." << endl);
            }

            node = node->next_sibling();
        }

        return result;
    }

    ManipulationObjectPtr ObjectIO::processManipulationObject(rapidxml::xml_node<char>* objectXMLNode, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!objectXMLNode, "No <ManipulationObject> tag in XML definition");

        bool visuProcessed = false;
        bool colProcessed = false;
        VisualizationNodePtr visualizationNode;
        CollisionModelPtr collisionModel;
        bool useAsColModel = false;
        SceneObject::Physics physics;
        bool physicsDefined = false;
        std::vector<GraspSetPtr> graspSets;
        Eigen::Matrix4f globalPose = Eigen::Matrix4f::Identity();

        // get name
        std::string objName = processNameAttribute(objectXMLNode);

        // first check if there is an xml file to load
        rapidxml::xml_node<>* xmlFileNode = objectXMLNode->first_node("file", 0, false);

        if (xmlFileNode)
        {
            std::string xmlFile = processFileNode(xmlFileNode, basePath);
            ManipulationObjectPtr result = loadManipulationObject(xmlFile);

            if (!result)
            {
                return result;
            }

            if (!objName.empty())
            {
                result->setName(objName);
            }

            // update global pose
            rapidxml::xml_node<>* poseNode = objectXMLNode->first_node("globalpose", 0, false);

            if (poseNode)
            {
                processTransformNode(poseNode, objName, globalPose);
                result->setGlobalPose(globalPose);
            }

            return result;
        }



        THROW_VR_EXCEPTION_IF(objName.empty(), "ManipulationObject definition expects attribute 'name'");

        rapidxml::xml_node<>* node = objectXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "visualization")
            {
                THROW_VR_EXCEPTION_IF(visuProcessed, "Two visualization tags defined in ManipulationObject '" << objName << "'." << endl);
                visualizationNode = processVisualizationTag(node, objName, basePath, useAsColModel);
                visuProcessed = true;

                if (useAsColModel)
                {
                    THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in ManipulationObject '" << objName << "'." << endl);
                    std::string colModelName = objName;
                    colModelName += "_VISU_ColModel";
                    // todo: ID?
                    collisionModel.reset(new CollisionModel(visualizationNode, colModelName, CollisionCheckerPtr()));
                    colProcessed = true;
                }
            }
            else if (nodeName == "collisionmodel")
            {
                THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in ManipulationObject '" << objName << "'." << endl);
                collisionModel = processCollisionTag(node, objName, basePath);
                colProcessed = true;
            }
            else if (nodeName == "physics")
            {
                THROW_VR_EXCEPTION_IF(physicsDefined, "Two physics tags defined in ManipulationObject '" << objName << "'." << endl);
                processPhysicsTag(node, objName, physics);
                physicsDefined = true;
            }
            else if (nodeName == "graspset")
            {
                GraspSetPtr gs = processGraspSet(node, objName);
                THROW_VR_EXCEPTION_IF(!gs, "Invalid grasp set in '" << objName << "'." << endl);
                graspSets.push_back(gs);

            }
            else if (nodeName == "globalpose")
            {
                processTransformNode(node, objName, globalPose);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in ManipulationObject <" << objName << ">." << endl);
            }

            node = node->next_sibling();
        }



        // build object
        ManipulationObjectPtr object(new ManipulationObject(objName, visualizationNode, collisionModel, physics));

        for (size_t i = 0; i < graspSets.size(); i++)
        {
            object->addGraspSet(graspSets[i]);
        }

        object->setGlobalPose(globalPose);

        return object;
    }

    ObstaclePtr ObjectIO::processObstacle(rapidxml::xml_node<char>* objectXMLNode, const std::string& basePath)
    {
        THROW_VR_EXCEPTION_IF(!objectXMLNode, "No <Obstacle> tag in XML definition");

        bool visuProcessed = false;
        bool colProcessed = false;
        VisualizationNodePtr visualizationNode;
        CollisionModelPtr collisionModel;
        bool useAsColModel = false;
        SceneObject::Physics physics;
        bool physicsDefined = false;
        Eigen::Matrix4f globalPose = Eigen::Matrix4f::Identity();

        // get name
        std::string objName = processNameAttribute(objectXMLNode);

        // first check if there is an xml file to load
        rapidxml::xml_node<>* xmlFileNode = objectXMLNode->first_node("file", 0, false);

        if (xmlFileNode)
        {
            std::string xmlFile = processFileNode(xmlFileNode, basePath);
            ObstaclePtr result = loadObstacle(xmlFile);

            if (!result)
            {
                return result;
            }

            if (!objName.empty())
            {
                result->setName(objName);
            }

            // update global pose
            rapidxml::xml_node<>* poseNode = objectXMLNode->first_node("globalpose", 0, false);

            if (poseNode)
            {
                processTransformNode(poseNode, objName, globalPose);
                result->setGlobalPose(globalPose);
            }

            return result;
        }

        THROW_VR_EXCEPTION_IF(objName.empty(), "Obstacle definition expects attribute 'name'");


        rapidxml::xml_node<>* node = objectXMLNode->first_node();

        while (node)
        {
            std::string nodeName = getLowerCase(node->name());

            if (nodeName == "visualization")
            {
                THROW_VR_EXCEPTION_IF(visuProcessed, "Two visualization tags defined in Obstacle '" << objName << "'." << endl);
                visualizationNode = processVisualizationTag(node, objName, basePath, useAsColModel);
                visuProcessed = true;

                if (useAsColModel)
                {
                    THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in Obstacle '" << objName << "'." << endl);
                    std::string colModelName = objName;
                    colModelName += "_VISU_ColModel";
                    // todo: ID?
                    collisionModel.reset(new CollisionModel(visualizationNode, colModelName, CollisionCheckerPtr()));
                    colProcessed = true;
                }
            }
            else if (nodeName == "collisionmodel")
            {
                THROW_VR_EXCEPTION_IF(colProcessed, "Two collision tags defined in Obstacle '" << objName << "'." << endl);
                collisionModel = processCollisionTag(node, objName, basePath);
                colProcessed = true;
            }
            else if (nodeName == "physics")
            {
                THROW_VR_EXCEPTION_IF(physicsDefined, "Two physics tags defined in Obstacle '" << objName << "'." << endl);
                processPhysicsTag(node, objName, physics);
                physicsDefined = true;
            }
            else if (nodeName == "globalpose")
            {
                processTransformNode(node, objName, globalPose);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Obstacle <" << objName << ">." << endl);
            }

            node = node->next_sibling();
        }

        // build object
        ObstaclePtr object(new Obstacle(objName, visualizationNode, collisionModel, physics));
        object->setGlobalPose(globalPose);
        return object;
    }



    VirtualRobot::ManipulationObjectPtr ObjectIO::createManipulationObjectFromString(const std::string& xmlString, const std::string& basePath /*= ""*/)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::ManipulationObjectPtr obj;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags
            rapidxml::xml_node<char>* objectXMLNode = doc.first_node("ManipulationObject");

            obj = processManipulationObject(objectXMLNode, basePath);



        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse data in xml definition" << endl
                               << "Error message:" << e.what() << endl
                               << "Position: " << endl << e.where<char>() << endl);
            return ManipulationObjectPtr();
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
            return ManipulationObjectPtr();
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl);
            return ManipulationObjectPtr();
        }

        delete[] y;
        return obj;
    }


    VirtualRobot::ObstaclePtr ObjectIO::createObstacleFromString(const std::string& xmlString, const std::string& basePath /*= ""*/)
    {
        // copy string content to char array
        char* y = new char[xmlString.size() + 1];
        strncpy(y, xmlString.c_str(), xmlString.size() + 1);

        VirtualRobot::ObstaclePtr obj;

        try
        {
            rapidxml::xml_document<char> doc;    // character type defaults to char
            doc.parse<0>(y);    // 0 means default parse flags
            rapidxml::xml_node<char>* objectXMLNode = doc.first_node("Obstacle");

            obj = processObstacle(objectXMLNode, basePath);



        }
        catch (rapidxml::parse_error& e)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Could not parse data in xml definition" << endl
                               << "Error message:" << e.what() << endl
                               << "Position: " << endl << e.where<char>() << endl);
            return ObstaclePtr();
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
            return ObstaclePtr();
        }
        catch (...)
        {
            delete[] y;
            THROW_VR_EXCEPTION("Error while parsing xml definition" << endl);
            return ObstaclePtr();
        }

        delete[] y;
        return obj;
    }

    bool ObjectIO::saveManipulationObject(ManipulationObjectPtr object, const std::string& xmlFile)
    {
        THROW_VR_EXCEPTION_IF(!object, "NULL object");

        boost::filesystem::path filenameBaseComplete(xmlFile);
        boost::filesystem::path filenameBasePath = filenameBaseComplete.branch_path();
        std::string basePath = filenameBasePath.string();

        std::string xmlString = object->toXML(basePath);

        // save file
        return BaseIO::writeXMLFile(xmlFile, xmlString, true);
    }


} // namespace VirtualRobot
