
#include "ContactSensor.h"
#include "ContactSensorFactory.h"
#include "../XML/BaseIO.h"

using namespace boost;

namespace VirtualRobot
{

    ContactSensor::ContactSensor(RobotNodeWeakPtr robotNode,
                                 const std::string& name)
        : Sensor(robotNode, name)
        , timestamp(0.0)
    {
    }


    ContactSensor::~ContactSensor()
    {
    }

    void ContactSensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            cout << "******** ContactSensor ********" << endl;
        }

        Sensor::print(printChildren, false);
    }


    SensorPtr ContactSensor::_clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel, float scaling)
    {
        SensorPtr result(new ContactSensor(newRobotNode, name));
        return result;
    }

    std::string ContactSensor::toXML(const std::string& modelPath, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor type='" << ContactSensorFactory::getName() << "'/>" << endl;
        std::string pre2 = pre + t;

        return ss.str();
    }

    void ContactSensor::updateSensors(const ContactSensor::ContactFrame& frame, double dt)
    {
        this->frame = frame;
        timestamp += dt;
    }

} // namespace VirtualRobot
