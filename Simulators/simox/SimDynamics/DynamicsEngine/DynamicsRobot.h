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
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _SimDynamics_DynamicsRobot_h_
#define _SimDynamics_DynamicsRobot_h_

#include "../SimDynamics.h"
#include "DynamicsObject.h"
#include "DynamicsUtils.h"
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/Sensor.h>


namespace SimDynamics
{
    class SIMDYNAMICS_IMPORT_EXPORT DynamicsRobot
    {
        friend class DynamicsEngine;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
        */
        DynamicsRobot(VirtualRobot::RobotPtr rob);

        /*!
        */
        virtual ~DynamicsRobot();

        std::string getName() const;

        VirtualRobot::RobotPtr getRobot()
        {
            return robot;
        }

        bool hasDynamicsRobotNode(VirtualRobot::RobotNodePtr node);
        std::vector<DynamicsObjectPtr> getDynamicsRobotNodes();

        /*!
            Returns dynamic model of node.
            An empty DynamicsObjectPtr is returned in case no dynamic version has been created so far.
        */
        DynamicsObjectPtr getDynamicsRobotNode(VirtualRobot::RobotNodePtr node);

        /*!
            Enable joint actuation for given node.
        */
        virtual void actuateNode(VirtualRobot::RobotNodePtr node, double jointValue, double jointVelocity);
        virtual void actuateNode(VirtualRobot::RobotNodePtr node, double jointValue);
        virtual void actuateNodeVel(VirtualRobot::RobotNodePtr node, double jointVelocity);
        virtual void actuateNodeTorque(VirtualRobot::RobotNodePtr node, double jointTorque);
        virtual void actuateNode(const std::string& node, double jointValue);
        virtual void actuateNodeVel(const std::string& node, double jointVelocity);
        virtual void actuateNodeTorque(const std::string& node, double jointTorque);
        virtual void disableNodeActuation(VirtualRobot::RobotNodePtr node);
        virtual bool isNodeActuated(VirtualRobot::RobotNodePtr node);
        virtual double getNodeTarget(VirtualRobot::RobotNodePtr node);
        virtual void enableActuation(ActuationMode mode);
        virtual void disableActuation();

        /*!
            Usually this method is called by the framework in every tick to perform joint actuation.
            \param dt Timestep
        */
        virtual void actuateJoints(double dt);
        virtual void updateSensors(double dt) {}

        // experimental...
        virtual void ensureKinematicConstraints();

        // We do not allow to re-adjust the robot.
        // The position of the robot is queried once on construction.
        // Then the physics simulation takes over.
        //virtual void setPosition(const Eigen::Vector3f &posMM);
        //virtual void setPose(const Eigen::Matrix4f &pose);


        virtual double getJointAngle(VirtualRobot::RobotNodePtr rn);
        virtual double getJointSpeed(VirtualRobot::RobotNodePtr rn);
        virtual double getJointTargetSpeed(VirtualRobot::RobotNodePtr rn);

        virtual Eigen::Matrix4f getComGlobal(const VirtualRobot::RobotNodePtr& rn);

        virtual Eigen::Vector3f getComGlobal(const VirtualRobot::RobotNodeSetPtr& bodies);
        virtual Eigen::Vector3f getComVelocityGlobal(const VirtualRobot::RobotNodeSetPtr& bodies);

        virtual Eigen::Vector3f getLinearMomentumGlobal(const VirtualRobot::RobotNodeSetPtr& set);
        virtual Eigen::Vector3f getAngularMomentumGlobal(const VirtualRobot::RobotNodeSetPtr& set);
        virtual Eigen::Vector3f getAngularMomentumLocal(const VirtualRobot::RobotNodeSetPtr& set);

        virtual void setGlobalPose(const Eigen::Matrix4f &gp);

        //! If set, all actions are protected with this mutex
        virtual void setMutex(boost::shared_ptr <boost::recursive_mutex> engineMutexPtr);

        //! can be used to access the internal controllers
        std::map<VirtualRobot::RobotNodePtr, VelocityMotorController>& getControllers();

        typedef boost::shared_ptr< boost::recursive_mutex::scoped_lock > MutexLockPtr;
        /*!
            This lock can be used to protect data access. It locks the mutex until deletion.
            If no mutex was specified, an empty lock will be returned which does not protect the engine calls (this is the standard behavior).
            \see setMutex

            Exemplary usage:
            {
                MutexLockPtr lock = engine->getScopedLock();
                // now the mutex is locked

                // access data
                // ...

            } // end of scope -> lock gets deleted and mutex is released automatically
        */
        MutexLockPtr getScopedLock();
    protected:

        virtual void createDynamicsNode(VirtualRobot::RobotNodePtr node);

        //! creates a link and attaches object to internal data structure
        virtual bool attachObject(const std::string& nodeName, DynamicsObjectPtr object);
        virtual bool detachObject(DynamicsObjectPtr object);


        struct robotNodeActuationTarget
        {
            robotNodeActuationTarget()
                : jointValueTarget(0)
                , jointVelocityTarget(0)
                , jointTorqueTarget(0)
            {
                actuation.mode = 0;
            }
            double jointValueTarget;
            double jointVelocityTarget;
            double jointTorqueTarget;
            VirtualRobot::RobotNodePtr node;
            //DynamicsObjectPtr dynNode; // if node is a joint without model, there is no dyn node!
            ActuationMode actuation;
        };

        std::map<VirtualRobot::RobotNodePtr, robotNodeActuationTarget> actuationTargets;
        std::map<VirtualRobot::RobotNodePtr, VelocityMotorController> actuationControllers;

        VirtualRobot::RobotPtr robot;

        std::vector<VirtualRobot::SensorPtr> sensors;

        std::vector<VirtualRobot::RobotNodePtr> robotNodes;
        std::map<VirtualRobot::RobotNodePtr, DynamicsObjectPtr> dynamicRobotNodes;

        boost::shared_ptr <boost::recursive_mutex> engineMutexPtr;
    };

    typedef boost::shared_ptr<DynamicsRobot> DynamicsRobotPtr;

} // namespace SimDynamics

#endif // _SimDynamics_DynamicsRobot_h_
