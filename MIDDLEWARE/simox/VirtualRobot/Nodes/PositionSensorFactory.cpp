

#include "PositionSensorFactory.h"
#include "Sensor.h"
#include "../XML/BaseIO.h"
#include "../XML/rapidxml.hpp"
#include "PositionSensor.h"


namespace VirtualRobot
{

    PositionSensorFactory::PositionSensorFactory()
    {
    }


    PositionSensorFactory::~PositionSensorFactory()
    {
    }


    /**
     * This method creates a VirtualRobot::PositionSensor.
     *
     * \return instance of VirtualRobot::PositionSensor.
     */
    SensorPtr PositionSensorFactory::createSensor(RobotNodePtr node, const std::string& name, VisualizationNodePtr visualization,
            const Eigen::Matrix4f& rnTrafo) const
    {
        SensorPtr Sensor(new PositionSensor(node, name, visualization, rnTrafo));

        return Sensor;
    }

    SensorPtr PositionSensorFactory::createSensor(RobotNodePtr node, rapidxml::xml_node<char>* sensorXMLNode, RobotIO::RobotDescription loadMode, const std::string basePath) const
    {
        THROW_VR_EXCEPTION_IF(!sensorXMLNode, "NULL data");
        THROW_VR_EXCEPTION_IF(!node, "NULL data");


        // get name
        std::string sensorName = BaseIO::processNameAttribute(sensorXMLNode, true);

        if (sensorName.empty())
        {
            std::stringstream ss;
            ss << node->getName() << "_PositionSensor";
            sensorName = ss.str();
        }


        // visu data
        bool visuProcessed = false;
        bool enableVisu = true;
        bool useAsColModel = false;

        VisualizationNodePtr visualizationNode;

        Eigen::Matrix4f transformMatrix = Eigen::Matrix4f::Identity();

        rapidxml::xml_node<>* nodeXML = sensorXMLNode->first_node();

        while (nodeXML)
        {
            std::string nodeName = BaseIO::getLowerCase(nodeXML->name());

            if (nodeName == "visualization")
            {
                if (loadMode == RobotIO::eFull)
                {
                    THROW_VR_EXCEPTION_IF(visuProcessed, "Two visualization tags defined in Sensor '" << sensorName << "'." << endl);
                    visualizationNode = BaseIO::processVisualizationTag(nodeXML, sensorName, basePath, useAsColModel);
                    visuProcessed = true;
                }// else silently ignore tag
            }
            else if (nodeName == "transform")
            {
                BaseIO::processTransformNode(sensorXMLNode, sensorName, transformMatrix);
            }
            else
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Sensor <" << sensorName << ">." << endl);
            }

            nodeXML = nodeXML->next_sibling();
        }



        SensorPtr Sensor(new PositionSensor(node, sensorName, visualizationNode, transformMatrix));

        return Sensor;
    }


    /**
     * register this class in the super class factory
     */
    SensorFactory::SubClassRegistry PositionSensorFactory::registry(PositionSensorFactory::getName(), &PositionSensorFactory::createInstance);


    /**
     * \return "position"
     */
    std::string PositionSensorFactory::getName()
    {
        return "position";
    }


    /**
     * \return new instance of PositionSensorFactory.
     */
    boost::shared_ptr<SensorFactory> PositionSensorFactory::createInstance(void*)
    {
        boost::shared_ptr<PositionSensorFactory> psFactory(new PositionSensorFactory());
        return psFactory;
    }

} // namespace VirtualRobot
