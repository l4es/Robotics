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
#ifndef _SimDynamics_SimoxMotionState_h_
#define _SimDynamics_SimoxMotionState_h_

#include "../../SimDynamics.h"
#include "../DynamicsObject.h"

#include <btBulletCollisionCommon.h>

#include <VirtualRobot/Nodes/RobotNodeActuator.h>
#include <VirtualRobot/SceneObject.h>

#include <vector>

namespace SimDynamics
{



    /*!
        A derived class can be added to a SimoxMotionState and it will get notified whenever the motion is updated.
        To use this class you have to implement the poseChanged method in a derived class.
    */
    struct SIMDYNAMICS_IMPORT_EXPORT SimoxMotionStateCallback
    {
        SimoxMotionStateCallback() {}
        virtual ~SimoxMotionStateCallback() {}
        virtual void poseChanged(const btTransform& worldPose) = 0;
    };

#ifdef _WIN32
#pragma warning(disable: 4275)
#endif

    class SIMDYNAMICS_IMPORT_EXPORT SimoxMotionState : public btDefaultMotionState
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*!
            Constructor.
            The object is queried for current globalpose and center of mass.

            \param sceneObject The obvject.
        */
        SimoxMotionState(VirtualRobot::SceneObjectPtr sceneObject);
        /*!
            Destructor.
        */
        virtual ~SimoxMotionState();

        /*!
            Interface for changing poses of rigid body and VirtualRobot objects.

            Bullet sets and gets the rigid body world pose using
            setWorldTransform() and getWorldTransform().
        */
        virtual void setWorldTransform(const btTransform& worldPose);

        /*!
            Interface for getting poses of rigid body and VirtualRobot objects.

            Bullet sets and gets the rigid body world pose using
            setWorldTransform() and getWorldTransform().
        */
        virtual void getWorldTransform(btTransform& worldPose) const;

        /*!
            Set center of mass.
            Since Bullet assumes that the local origin is equal to its COM but in Simox this is not assumed
            we have to take care of a local COM transformation.

            \param com The com position in local coordinate system (corresponds to the origin in bullet).
        */
        void setCOM(const Eigen::Vector3f& com);
        Eigen::Vector3f getCOM() const;

        /*!
            Derive a class from SimoxMotionStateCallback, overwrite the poseChanged method and register it
            to this SimoxMotionState.
            The callbeck will be executed at the beginning of the setWorldTransform() function.
        */
        void registerCallback(SimoxMotionStateCallback* cb);
        std::vector<SimoxMotionStateCallback*> getCallbacks();

        /*!
            Set the pose.
        */
        void setGlobalPose(const Eigen::Matrix4f& pose);

    protected:
        void updateTransform();
        void _setCOM(const Eigen::Vector3f& com);

        void setGlobalPoseSimox(const Eigen::Matrix4f& worldPose);

        Eigen::Matrix4f initalGlobalPose;
        Eigen::Vector3f com;

        VirtualRobot::SceneObjectPtr sceneObject;
        VirtualRobot::RobotNodeActuatorPtr robotNodeActuator; // in case sceneObject is of type RobotNode, we  can actuate it with this actuator.

        // This is the bullet transformation.
        btTransform _transform;
        btTransform _graphicsTransfrom;
        btTransform _comOffset;


        std::vector<SimoxMotionStateCallback*> callbacks;
    };

}


#endif // _SimDynamics_SimoxMotionState_h_

