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
#ifndef _SimDynamics_BulletObject_h_
#define _SimDynamics_BulletObject_h_

#include "../DynamicsObject.h"
#include "SimoxMotionState.h"

#include <btBulletDynamicsCommon.h>

namespace SimDynamics
{
    class SIMDYNAMICS_IMPORT_EXPORT BulletObject : public DynamicsObject
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
        */
        BulletObject(VirtualRobot::SceneObjectPtr o);

        /*!
        */
        virtual ~BulletObject();


        boost::shared_ptr<btRigidBody> getRigidBody();


        /*!
            Set world position [MM].
        */
        virtual void setPosition(const Eigen::Vector3f& posMM);

        /*!
            Set world pose [mm].
        */
        virtual void setPose(const Eigen::Matrix4f& pose);

        Eigen::Vector3f getCom()
        {
            return com;
        }

        virtual Eigen::Vector3f getLinearVelocity();
        virtual Eigen::Vector3f getAngularVelocity();

        virtual void setLinearVelocity(const Eigen::Vector3f& vel);
        virtual void setAngularVelocity(const Eigen::Vector3f& vel);

        //! This is the world pose which is set by bullet
        Eigen::Matrix4f getComGlobal();

        /*!
         * \brief applyForce Applies an external force on this object. The force is applied at the CoM position.
         * \param force The force to apply (value with respect to one second). The force will be deleted after one simulation step.
         */
        virtual void applyForce(const Eigen::Vector3f& force);

        /*!
         * \brief applyTorque Applies an external torque on this object. The torque is applied at the CoM position.
         * \param torque The torque to apply (value with respect to one second). The torque will be deleted after one simulation step.
         */
        virtual void applyTorque(const Eigen::Vector3f& torque);

        virtual void setSimType(VirtualRobot::SceneObject::Physics::SimulationType s);

    protected:

        void setPoseIntern(const Eigen::Matrix4f& pose);
        btCollisionShape* getShapeFromPrimitive(VirtualRobot::Primitive::PrimitivePtr primitive);

        btConvexHullShape* createConvexHullShape(VirtualRobot::TriMeshModelPtr trimesh);

        boost::shared_ptr<btRigidBody> rigidBody;
        boost::shared_ptr<btCollisionShape> collisionShape; // bullet collision shape

        Eigen::Vector3f com; // com offset of trimesh

        btScalar btMargin;
        SimoxMotionState* motionState;

    };

    typedef boost::shared_ptr<BulletObject> BulletObjectPtr;

} // namespace SimDynamics

#endif // _SimDynamics_BulletObject_h_
