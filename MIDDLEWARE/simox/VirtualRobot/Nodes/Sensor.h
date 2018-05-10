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
#ifndef _VirtualRobot_Sensor_h_
#define _VirtualRobot_Sensor_h_

#include "../VirtualRobotImportExport.h"
#include "../VirtualRobotException.h"
#include "../VirtualRobot.h"

#include "../SceneObject.h"
#include "../Visualization/VisualizationNode.h"

#include <Eigen/Core>
#include <Eigen/Geometry>



#include <string>
#include <vector>


namespace VirtualRobot
{

    class Sensor;
    typedef boost::shared_ptr<Sensor> SensorPtr;


    /*!
        A sensor can be attached to a RobotNode.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Sensor : public SceneObject
    {
    public:
        friend class Robot;
        friend class RobotIO;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor with settings.
        */
        Sensor(RobotNodeWeakPtr robotNode,
               const std::string& name,
               VisualizationNodePtr visualization = VisualizationNodePtr(),
               const Eigen::Matrix4f& rnTrafo = Eigen::Matrix4f::Identity()
              );

        /*!
        */
        virtual ~Sensor();


        RobotNodePtr getRobotNode() const;


        /*!
            The transformation that specifies the pose of the sensor relatively to the pose of the parent RobotNode.
        */
        virtual Eigen::Matrix4f getRobotNodeToSensorTransformation()
        {
            return rnTransformation;
        }

        /*!
            Calling this SceneObject method will cause an exception, since Sensors are controlled via their RobotNode parent.
        */
        virtual void setGlobalPose(const Eigen::Matrix4f& pose);

        /*!
            Print status information.
        */
        virtual void print(bool printChildren = false, bool printDecoration = true) const;


        /*!
            Clone this Sensor.
            \param newRobotNode The newly created Sensor belongs to newRobotNode.
            \param scaling Scales the visualization and transformation data.
        */
        virtual SensorPtr clone(RobotNodePtr newRobotNode, float scaling = 1.0f);


        //! Forbid cloning method from SceneObject. We need to know the new robotnode for cloning
        SceneObjectPtr clone(const std::string& name, CollisionCheckerPtr colChecker = CollisionCheckerPtr(), float scaling = 1.0f) const
        {
            THROW_VR_EXCEPTION("Cloning not allowed this way...");
        }

        /*!
            Compute/Update the transformations of this sensor. Therefore the parent is queried for its pose.
        */
        virtual void updatePose(bool updateChildren = true);

        virtual bool initialize(SceneObjectPtr parent = SceneObjectPtr(), const std::vector<SceneObjectPtr>& children = std::vector<SceneObjectPtr>());

        virtual std::string toXML(const std::string& modelPath, int tabs = 1);

    protected:


        /*!
            Update the pose according to parent pose
        */
        virtual void updatePose(const Eigen::Matrix4f& parentPose, bool updateChildren = true);

        Sensor() {};


        Eigen::Matrix4f rnTransformation;           //<! Transformation from parent's coordinate system to this sensor

        RobotNodeWeakPtr robotNode;

        /*!
            Derived classes must implement their clone method here.
            The visualization is already scaled, the kinematic information (i.e. transformations) have to be scaled by derived implementations.
        */
        virtual SensorPtr _clone(const RobotNodePtr newRobotNode, const VisualizationNodePtr visualizationModel, float scaling) = 0;

        virtual SceneObject* _clone(const std::string& name, CollisionCheckerPtr colChecker = CollisionCheckerPtr(), float scaling = 1.0f) const
        {
            THROW_VR_EXCEPTION("Cloning not allowed this way...");
        }

    };

} // namespace VirtualRobot

#endif // _VirtualRobot_Sensor_h_
