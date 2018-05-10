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
#ifndef _SimDynamics_BulletRobot_h_
#define _SimDynamics_BulletRobot_h_

#include "../DynamicsRobot.h"
#include "BulletObject.h"

#include <btBulletDynamicsCommon.h>

namespace SimDynamics
{
    class SIMDYNAMICS_IMPORT_EXPORT BulletRobot : public DynamicsRobot
    {
        friend class BulletEngine;
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor.
            Create a dynamic representation by building all related bullet objects.
        */
        BulletRobot(VirtualRobot::RobotPtr rob, bool enableJointMotors = true);

        /*!
        */
        virtual ~BulletRobot();

        struct LinkInfo
        {
            VirtualRobot::RobotNodePtr nodeA; // parent
            VirtualRobot::RobotNodePtr nodeB; // child
            VirtualRobot::RobotNodePtr nodeJoint; // joint
            VirtualRobot::RobotNodePtr nodeJoint2; // joint2 (only used for hinge2/universal joints)
            BulletObjectPtr dynNode1; // parent
            BulletObjectPtr dynNode2; // child
            std::vector< std::pair<DynamicsObjectPtr, DynamicsObjectPtr> > disabledCollisionPairs;
            boost::shared_ptr<btTypedConstraint> joint;
            double jointValueOffset; // offset simox -> bullet joint values
        };

        typedef boost::shared_ptr<LinkInfo> LinkInfoPtr;


        /**
         * @brief The force torque output needs to be activated before use
         * @param link Link for which the force torque should be enabled/disabled
         * @param enable
         */
        void enableForceTorqueFeedback(const LinkInfo& link, bool enable = true);

        /**
         * @brief getForceTorqueFeedbackA retrieves the force torque in the first body of the link
         * @param link  Link for which the force torque should be retrieved
         * @return 6 Dim. vector. First 3 values are the forces, last 3 are torques.
         * Values are in N and N*m. Position of the values is the center of mass of first body of the link
         * in the global coordinate system.
         */
        Eigen::VectorXf getForceTorqueFeedbackA(const LinkInfo& link);
        Eigen::VectorXf getForceTorqueFeedbackB(const LinkInfo& link);
        /**
         * @brief getJointForceTorqueGlobal retrieves the force torque in the joint between the bodies
         * @param link  Link for which the force torque should be retrieved
         * @return 6 Dim. vector. First 3 values are the forces, last 3 are torques.
         * Values are in N and N*m. Position of the values is in the middle of the joint
         * in the global coordinate system.
         */
        Eigen::VectorXf getJointForceTorqueGlobal(const LinkInfo& link);


        // We do not allow to re-adjust the robot.
        // The position of the robot is queried once on construction.
        // Then the physics simulation takes over.
        //virtual void setPosition(const Eigen::Vector3f &posMM);
        //virtual void setPose(const Eigen::Matrix4f &pose);

        bool hasLink(VirtualRobot::RobotNodePtr node1, VirtualRobot::RobotNodePtr node2);

        //! Returns true if the joint of node is covered by a link
        bool hasLink(VirtualRobot::RobotNodePtr node);

        std::vector<LinkInfo> getLinks();

        virtual void actuateNode(VirtualRobot::RobotNodePtr node, double jointValue);
        virtual void actuateNodeVel(VirtualRobot::RobotNodePtr node, double jointVelocity);

        /*!
            Usually this method is called by the framework in every tick to perform joint actuation.
            \param dt Timestep
        */
        virtual void actuateJoints(double dt);
        virtual void updateSensors(double dt);

        virtual double getJointAngle(VirtualRobot::RobotNodePtr rn);
        virtual double getJointSpeed(VirtualRobot::RobotNodePtr rn);
        virtual double getJointTargetSpeed(VirtualRobot::RobotNodePtr rn);
        virtual double getNodeTarget(VirtualRobot::RobotNodePtr node);

        /*!
         * \brief getJointTorques retrieves the torques in the given joint.
         * \param rn
         * \return Returns the torques in the given joint. If rn is not a
         * rotational joint zero is returned.
         * Values are in N*m. Position of the values is in the middle of the joint
         * in the global coordinate system.
         */
        Eigen::Vector3f getJointTorques(VirtualRobot::RobotNodePtr rn);

        /*!
         * \brief getJointTorque retrieves the torque along the axis in the given joint.
         * \param rn
         * \return Returns the torque in the given joint. If rn is not a
         * rotational joint zero is returned.
         * Values are in N*m.
         */
        double getJointTorque(VirtualRobot::RobotNodePtr rn);

        /*!
         * \brief getJointForce retrieves the forces in the given joint.
         * \param rn
         * \return Returns the forces in the given joint. If rn is not a
         * rotational joint zero is returned.
         * Values are in N. Position of the values is in the middle of the joint
         * in the global coordinate system.
         */
        Eigen::Vector3f getJointForces(VirtualRobot::RobotNodePtr rn);

        /*!
            Returns the CoM pose, which is reported by bullet
        */
        virtual Eigen::Matrix4f getComGlobal(const VirtualRobot::RobotNodePtr& rn);
        virtual Eigen::Vector3f getComGlobal(const VirtualRobot::RobotNodeSetPtr& set);
        virtual Eigen::Vector3f getComVelocityGlobal(const VirtualRobot::RobotNodeSetPtr& set);

        /*!
         * Returns the linear momentum in Ns for the bodies in the nodeset.
         */
        virtual Eigen::Vector3f getLinearMomentumGlobal(const VirtualRobot::RobotNodeSetPtr& set);

        /*!
         * Returns the angular momentum in Nms for the bodies in the nodeset
         */
        virtual Eigen::Vector3f getAngularMomentumGlobal(const VirtualRobot::RobotNodeSetPtr& set);

        /*!
         * Returns the angular momentum in Nms for the bodies in the nodeset relative to the CoM
         */
        virtual Eigen::Vector3f getAngularMomentumLocal(const VirtualRobot::RobotNodeSetPtr& set);

        // experimental...
        virtual void ensureKinematicConstraints();

        /*!
            Returns link where the given node is the joint node.
        */
        LinkInfo getLink(VirtualRobot::RobotNodePtr node);

        //! Get link that connects both objects
        LinkInfo getLink(BulletObjectPtr object1, BulletObjectPtr object2);

        /*!
            Returns all links where the given node is involved (bodyA, bodyB or joint)
        */
        std::vector<LinkInfo> getLinks(VirtualRobot::RobotNodePtr node);

        /*!
            Returns all links where the given node is involved (dynNode1 or dynNode2)
        */
        std::vector<LinkInfo> getLinks(BulletObjectPtr node);

        /*!
         * Sets the maxium motor impulse. Since this value
         * is used as limit *per simulation step* it depends heavily
         * on the duration of the simulation step.
         */
        void setMaximumMotorImpulse(double maxImpulse);

        /*!
         * Returns the currently used maxium motor impulse.
         */
        double getMaximumMotorImpulse() const;

    protected:
        void buildBulletModels(bool enableJointMotors);

        //! creates a link and attaches object to internal data structure
        virtual bool attachObject(const std::string& nodeName, DynamicsObjectPtr object);
        LinkInfoPtr attachObjectLink(const std::string& nodeName, DynamicsObjectPtr object);

        virtual bool detachObject(DynamicsObjectPtr object);


        //void createLink( VirtualRobot::RobotNodePtr bodyA, VirtualRobot::RobotNodePtr joint, VirtualRobot::RobotNodePtr bodyB, Eigen::Matrix4f &trafoA2J, Eigen::Matrix4f &trafoJ2B, bool enableJointMotors = true );
        // Possible joint types:
        // fixed                (joint=fixed        !joint2)
        // hinge                (joint=revolute     !joint2)
        // universal (hinge2)   (joint=revolute     joint2=revolute) // experimental
        void createLink(VirtualRobot::RobotNodePtr bodyA, VirtualRobot::RobotNodePtr joint, VirtualRobot::RobotNodePtr joint2, VirtualRobot::RobotNodePtr bodyB, bool enableJointMotors = true);

        void createLink(VirtualRobot::RobotNodePtr node1, VirtualRobot::RobotNodePtr node2, bool enableJointMotors);

        // ensure that all robot nodes, which are not actuated directly, are at the correct pose
        void setPoseNonActuatedRobotNodes();

        // process all ignoreCollision tags of physics section of RobotNode. Adds according collision disabled information to physics engine.
        void addIgnoredCollisionModels(VirtualRobot::RobotNodePtr rn);

        // removes all links in list where dynNode1 and dynNode2 are as given in l
        bool removeLink(const LinkInfo& l);

        std::vector<LinkInfo> links;

        btScalar bulletMaxMotorImulse;
        boost::shared_ptr<btTypedConstraint> createFixedJoint(boost::shared_ptr<btRigidBody> btBody1, boost::shared_ptr<btRigidBody> btBody2, Eigen::Matrix4f& anchor_inNode1, Eigen::Matrix4f& anchor_inNode2);
        boost::shared_ptr<btTypedConstraint> createHingeJoint(boost::shared_ptr<btRigidBody> btBody1, boost::shared_ptr<btRigidBody> btBody2, Eigen::Matrix4f& coordSystemNode1, Eigen::Matrix4f& coordSystemNode2,  Eigen::Matrix4f& anchor_inNode1, Eigen::Matrix4f& anchor_inNode2, Eigen::Vector3f& axisGlobal, Eigen::Vector3f& axisLocal, Eigen::Matrix4f& coordSystemJoint, double limMinBT, double limMaxBT);
    };

    typedef boost::shared_ptr<BulletRobot> BulletRobotPtr;

} // namespace SimDynamics

#endif // _SimDynamics_BulletRobot_h_
