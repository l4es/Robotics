
#include "PositionSensor.h"
#include "PositionSensorFactory.h"
#include "../XML/BaseIO.h"

using namespace boost;

namespace VirtualRobot
{

    PositionSensor::PositionSensor(RobotNodeWeakPtr robotNode,
                                   const std::string& name,
                                   VisualizationNodePtr visualization,
                                   const Eigen::Matrix4f& rnTrafo
                                  ) : Sensor(robotNode, name, visualization, rnTrafo)
    {

    }


    PositionSensor::~PositionSensor()
    {
    }



    void PositionSensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            cout << "******** PositionSensor ********" << endl;
        }

        Sensor::print(printChildren, false);
    }


    SensorPtr PositionSensor::_clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel, float scaling)
    {
        THROW_VR_EXCEPTION_IF(scaling < 0, "Scaling must be >0");
        Eigen::Matrix4f rnt = rnTransformation;
        rnt.block(0, 3, 3, 1) *= scaling;
        SensorPtr result(new PositionSensor(newRobotNode, name, visualizationModel, rnt));
        return result;
    }


    std::string PositionSensor::toXML(const std::string& modelPath, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor type='" << PositionSensorFactory::getName() << "' name='" << name << "'>" << endl;
        std::string pre2 = pre + t;
        std::string pre3 = pre2 + t;
        ss << pre2 << "<Transform>" << endl;
        ss << BaseIO::toXML(rnTransformation, pre3);
        ss << pre2 << "</Transform>" << endl;

        if (visualizationModel)
        {
            ss << visualizationModel->toXML(modelPath, tabs + 1);
        }

        ss << pre << "</Sensor>" << endl;
        return ss.str();
    }

} // namespace VirtualRobot
