

#include "ForceTorqueSensorFactory.h"
#include "Sensor.h"
#include "../XML/BaseIO.h"
#include "../XML/rapidxml.hpp"
#include "ForceTorqueSensor.h"


namespace VirtualRobot
{

    ForceTorqueSensorFactory::ForceTorqueSensorFactory()
    {
    }


    ForceTorqueSensorFactory::~ForceTorqueSensorFactory()
    {
    }


    /**
     * This method creates a VirtualRobot::ForceTorqueSensor.
     *
     * \return instance of VirtualRobot::ForceTorqueSensor.
     */
    SensorPtr ForceTorqueSensorFactory::createSensor(RobotNodePtr node, const std::string& name, VisualizationNodePtr visualization,
            const Eigen::Matrix4f& rnTrafo) const
    {
        SensorPtr Sensor(new ForceTorqueSensor(node, name));

        return Sensor;
    }

    SensorPtr ForceTorqueSensorFactory::createSensor(RobotNodePtr node, rapidxml::xml_node<char>* sensorXMLNode, RobotIO::RobotDescription loadMode, const std::string basePath) const
    {
        THROW_VR_EXCEPTION_IF(!sensorXMLNode, "NULL data");
        THROW_VR_EXCEPTION_IF(!node, "NULL data");


        // get name
        std::string sensorName = BaseIO::processNameAttribute(sensorXMLNode, true);

        if (sensorName.empty())
        {
            std::stringstream ss;
            ss << node->getName() << "_ForceTorqueSensor";
            sensorName = ss.str();
        }


        // visu data
        //bool visuProcessed = false;
        //bool enableVisu = true;
        //bool useAsColModel = false;

        //VisualizationNodePtr visualizationNode;

        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();

        rapidxml::xml_node<>* nodeXML = sensorXMLNode->first_node();

        while (nodeXML)
        {
            std::string nodeName = BaseIO::getLowerCase(nodeXML->name());
            //      if (nodeName == "visualization")
            //      {
            //          if (loadMode==RobotIO::eFull)
            //          {
            //              THROW_VR_EXCEPTION_IF(visuProcessed, "Two visualization tags defined in Sensor '" << sensorName << "'." << endl);
            //              visualizationNode = BaseIO::processVisualizationTag(nodeXML, sensorName, basePath, useAsColModel);
            //              visuProcessed = true;
            //          }// else silently ignore tag
            //      } else
            if (nodeName == "transform")
            {
                BaseIO::processTransformNode(sensorXMLNode,sensorName,transformMatrix);
            } else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Sensor <" << sensorName << ">." << endl);
            }

            nodeXML = nodeXML->next_sibling();
        }



        SensorPtr Sensor(new ForceTorqueSensor(node, sensorName, transformMatrix));

        return Sensor;
    }


    /**
     * register this class in the super class factory
     */
    SensorFactory::SubClassRegistry ForceTorqueSensorFactory::registry(ForceTorqueSensorFactory::getName(), &ForceTorqueSensorFactory::createInstance);


    /**
     * \return "position"
     */
    std::string ForceTorqueSensorFactory::getName()
    {
        return "forcetorque";
    }


    /**
     * \return new instance of ForceTorqueSensorFactory.
     */
    boost::shared_ptr<SensorFactory> ForceTorqueSensorFactory::createInstance(void*)
    {
        boost::shared_ptr<ForceTorqueSensorFactory> psFactory(new ForceTorqueSensorFactory());
        return psFactory;
    }

} // namespace VirtualRobot
