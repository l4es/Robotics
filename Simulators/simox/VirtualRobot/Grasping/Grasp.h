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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_Grasp_h_
#define _VirtualRobot_Grasp_h_

#include "../VirtualRobotImportExport.h"

#include <string>
#include <vector>

#include <VirtualRobot/EndEffector/EndEffector.h>
#include <Eigen/Core>

namespace VirtualRobot
{
    /*!
        A grasp is mainly specified by a TCP to object transformation.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Grasp
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            \param name The name of this grasp.
            \param robotType The robot type for which this grasp is defined.
            \param eef The end effector name.
            \param poseInTCPCoordSystem The pose of the object, given in eef's tcp coordinate system
            \param creation A custom string explaining how the grasp was created.
            \param quality A custom quality index.
            \param eefPreshape An optional preshape.
        */
        Grasp(const std::string& name, const std::string& robotType, const std::string& eef, const Eigen::Matrix4f& poseInTCPCoordSystem, const std::string& creation = std::string(""), float quality = 0.0f, const std::string& eefPreshape = std::string(""));

        /*!
        */
        virtual ~Grasp();

        void print(bool printDecoration = true);

        void setName(const std::string& name);
        void setPreshape(const std::string& preshapeName);

        /*!
            Get the (current) global pose of the target object when the grasp is applied on the corresponding EEF of robot.
            Note, that this pose is only valid for the current configuration of the robot.
            When the robot moves, the grasping pose will change, and you have to call this method again.
        */
        Eigen::Matrix4f getTargetPoseGlobal(RobotPtr robot);

        /*!
            Get the global pose that has to be achieved by the tcp in order to apply the grasp on the object at position objectPose.
        */
        Eigen::Matrix4f getTcpPoseGlobal(const Eigen::Matrix4f& objectPose);

        /*!
            The returned pose is the object pose that have to be set in order to apply a grasp at pose graspingPose.
        */
        Eigen::Matrix4f getObjectTargetPoseGlobal(const Eigen::Matrix4f& graspingPose);

        /*!
            Set the transformation of this grasp.
            The transformation is given in the coordinate system of the tcp (whereas the tcp belongs to the eef).
            \param tcp2Object The transformation specifies the tcp to object relation.
        */
        void setTransformation(const Eigen::Matrix4f& tcp2Object);

        std::string getName();
        std::string getRobotType();
        std::string getEefName();
        std::string getCreationMethod();
        std::string getPreshapeName();
        float getQuality();

        /*!
            Return the transformation of this grasp.
            The transformation is given in the coordinate system of the tcp (whereas the tcp belongs to the eef).
            This transformation specifies the tcp to object relation.
        */
        Eigen::Matrix4f getTransformation();

        std::string toXML(int tabs = 2);

        GraspPtr clone();

        /*!
            Returns the (optionally) stored configuration of the fingers / actors.
        */
        std::map< std::string, float > getConfiguration();

        /*!
            Optionally the configuration of the fingers / actors can be stored.

        */
        void setConfiguration(std::map< std::string, float >& c);

    protected:
        std::map< std::string, float > eefConfiguration; //!< Optional: the configuration of the actors.

        Eigen::Matrix4f poseTcp;    //!< The pose in TCP's coordinate system
        std::string robotType;
        std::string eef;            //!< The eef specifies which TCP node should be used
        std::string name;
        std::string creation;
        float quality;

        std::string preshape;       //!< Optionally an eef-preshape can be defined.
    };

} // namespace

#endif // _VirtualRobot_Grasp_h_
