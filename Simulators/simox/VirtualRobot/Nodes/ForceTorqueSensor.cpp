
#include "ForceTorqueSensor.h"
#include "ForceTorqueSensorFactory.h"
#include "../XML/BaseIO.h"

using namespace boost;

namespace VirtualRobot
{

    ForceTorqueSensor::ForceTorqueSensor(RobotNodeWeakPtr robotNode,
                                         const std::string& name,
                                         const Eigen::Matrix4f& rnTrafo) :
        Sensor(robotNode, name, VisualizationNodePtr(), rnTrafo),
        forceTorqueValues(6)
    {
        forceTorqueValues.setZero();
    }


    ForceTorqueSensor::~ForceTorqueSensor()
    {
    }

    Eigen::Vector3f ForceTorqueSensor::getForce() const
    {
        return forceTorqueValues.head(3);
    }

    Eigen::Vector3f ForceTorqueSensor::getTorque() const
    {
        return forceTorqueValues.tail(3);
    }

    const Eigen::VectorXf& ForceTorqueSensor::getForceTorque()
    {
        return forceTorqueValues;
    }

    Eigen::Vector3f ForceTorqueSensor::getAxisTorque()
    {
        Eigen::Vector3f torqueVector = forceTorqueValues.tail(3);

        // project onto joint axis
        //RobotNodePtr rn(robotNode);
        Eigen::Vector3f zAxis = this->globalPose.block(0, 2, 3, 1);
        Eigen::Vector3f axisTorque = (torqueVector.dot(zAxis)) * zAxis;

        return axisTorque;
    }



    void ForceTorqueSensor::print(bool printChildren, bool printDecoration) const
    {
        if (printDecoration)
        {
            cout << "******** ForceTorqueSensor ********" << endl;
        }

        Sensor::print(printChildren, false);
    }


    SensorPtr ForceTorqueSensor::_clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel, float scaling)
    {
        SensorPtr result(new ForceTorqueSensor(newRobotNode, name));
        return result;
    }


    std::string ForceTorqueSensor::toXML(const std::string& modelPath, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += t;
        }

        ss << pre << "<Sensor type='" << ForceTorqueSensorFactory::getName() << "' name='" << name << "'>" << endl;
        std::string pre2 = pre + t;
        ss << pre << "<Transform>" << endl;
        ss << BaseIO::toXML(rnTransformation, pre2);
        ss << pre << "</Transform>" << endl;


        ss << pre << "</Sensor>" << endl;
        return ss.str();
    }

    void ForceTorqueSensor::updateSensors(const Eigen::VectorXf& newForceTorque)
    {
        forceTorqueValues = newForceTorque;
    }

} // namespace VirtualRobot
