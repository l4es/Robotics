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
#ifndef _VirtualRobot_CameraSensor_h_
#define _VirtualRobot_CameraSensor_h_

#include "Sensor.h"


namespace VirtualRobot
{

    class CameraSensor;
    typedef boost::shared_ptr<CameraSensor> CameraSensorPtr;


    /*!
        A sensor for capturing simulated camera images.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CameraSensor : public Sensor
    {
    public:
        friend class Robot;
        friend class RobotIO;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor with settings.
        */
        CameraSensor(RobotNodeWeakPtr robotNode,
                     const std::string& name,
                     VisualizationNodePtr visualization = VisualizationNodePtr(),
                     const Eigen::Matrix4f& rnTrafo = Eigen::Matrix4f::Identity()
                    );

        /*!
        */
        virtual ~CameraSensor();


        /*!
            Print status information.
        */
        virtual void print(bool printChildren = false, bool printDecoration = true) const;


        virtual std::string toXML(const std::string& modelPath, int tabs);


    protected:


        CameraSensor() {};

        /*!
        Derived classes must implement their clone method here.
        */
        virtual SensorPtr _clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel, float scaling);

    };

} // namespace VirtualRobot

#endif // _VirtualRobot_CameraSensor_h_
