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
* @copyright  2013 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ForceTorqueSensor_h_
#define _VirtualRobot_ForceTorqueSensor_h_

#include "Sensor.h"


namespace VirtualRobot
{

    class ForceTorqueSensor;
    typedef boost::shared_ptr<ForceTorqueSensor> ForceTorqueSensorPtr;


    /*!
        This is a force torque sensor in a joint
        The force torque values are in world coordinates
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT ForceTorqueSensor : public Sensor
    {
    public:
        friend class Robot;
        friend class RobotIO;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor with settings.
        */
        ForceTorqueSensor(RobotNodeWeakPtr robotNode,
                          const std::string& name,
                          const Eigen::Matrix4f& rnTrafo = Eigen::Matrix4f::Identity()
                         );

        /*!
        */
        virtual ~ForceTorqueSensor();

        void updateSensors(const Eigen::VectorXf& newForceTorque);


        const Eigen::VectorXf& getForceTorque();
        Eigen::Vector3f getForce() const;
        Eigen::Vector3f getTorque() const;

        /**
         * Projects torque on joint axis
         */
        Eigen::Vector3f getAxisTorque();

        /*!
            Print status information.
        */
        virtual void print(bool printChildren = false, bool printDecoration = true) const;


        virtual std::string toXML(const std::string& modelPath, int tabs);

    protected:

        ForceTorqueSensor() {}


        /*!
        Derived classes must implement their clone method here.
        */
        virtual SensorPtr _clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel, float scaling);
        Eigen::VectorXf forceTorqueValues;
    };

    typedef boost::shared_ptr<ForceTorqueSensor> ForceTorqueSensorPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_ForceTorqueSensor_h_
