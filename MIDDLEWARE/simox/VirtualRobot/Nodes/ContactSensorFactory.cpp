

#include "ContactSensorFactory.h"
#include "Sensor.h"
#include "../XML/BaseIO.h"
#include "../XML/rapidxml.hpp"
#include "ContactSensor.h"


namespace VirtualRobot
{

    ContactSensorFactory::ContactSensorFactory()
    {
    }


    ContactSensorFactory::~ContactSensorFactory()
    {
    }


    /**
     * This method creates a VirtualRobot::ContactSensor.
     *
     * \return instance of VirtualRobot::ContactSensor.
     */
    SensorPtr ContactSensorFactory::createSensor(RobotNodePtr node, const std::string& name, VisualizationNodePtr visualization,
            const Eigen::Matrix4f& rnTrafo) const
    {
        SensorPtr Sensor(new ContactSensor(node, name));

        return Sensor;
    }

    SensorPtr ContactSensorFactory::createSensor(RobotNodePtr node, rapidxml::xml_node<char>* sensorXMLNode, RobotIO::RobotDescription loadMode, const std::string basePath) const
    {
        THROW_VR_EXCEPTION_IF(!sensorXMLNode, "NULL data");
        THROW_VR_EXCEPTION_IF(!node, "NULL data");


        // get name
        std::string sensorName = BaseIO::processNameAttribute(sensorXMLNode, true);

        if (sensorName.empty())
        {
            std::stringstream ss;
            ss << node->getName() << "_ContactSensor";
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
            {
                THROW_VR_EXCEPTION("XML definition <" << nodeName << "> not supported in Sensor <" << sensorName << ">." << endl);
            }

            nodeXML = nodeXML->next_sibling();
        }

        SensorPtr Sensor(new ContactSensor(node, sensorName));

        return Sensor;
    }


    /**
     * register this class in the super class factory
     */
    SensorFactory::SubClassRegistry ContactSensorFactory::registry(ContactSensorFactory::getName(), &ContactSensorFactory::createInstance);


    /**
     * \return "contact"
     */
    std::string ContactSensorFactory::getName()
    {
        return "contact";
    }


    /**
     * \return new instance of ContactSensorFactory.
     */
    boost::shared_ptr<SensorFactory> ContactSensorFactory::createInstance(void*)
    {
        boost::shared_ptr<ContactSensorFactory> psFactory(new ContactSensorFactory());
        return psFactory;
    }

} // namespace VirtualRobot
