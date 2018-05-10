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
#ifndef _SimDynamics_DynamicsEngine_h_
#define _SimDynamics_DynamicsEngine_h_

#include "../SimDynamics.h"
#include "DynamicsObject.h"
#include "DynamicsRobot.h"


namespace SimDynamics
{
    /*!
        Standard configuration for dynamics engines
    */
    class DynamicsEngineConfig
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DynamicsEngineConfig()
        {
            gravity << 0, 0, -9.81f;
        }

        virtual ~DynamicsEngineConfig() {}

        Eigen::Vector3f gravity;
    };


    typedef boost::shared_ptr<DynamicsEngineConfig> DynamicsEngineConfigPtr;

    /*!
        An interface class to encapsulates all calls to the underlying physics engine.
        Usually there is no need to instantiate this object by your own, it is automatically created when calling DynamicsWorld::Init().
    */
    class SIMDYNAMICS_IMPORT_EXPORT DynamicsEngine
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            \param engineMutex Optionally, all engine access methods can be protected by an external mutex. If not set, an internal mutex is creeated.
        */
        DynamicsEngine(boost::shared_ptr <boost::recursive_mutex> engineMutex = boost::shared_ptr<boost::recursive_mutex>());

        /*!
        */
        virtual ~DynamicsEngine();

        /*!
            Initialize the engine with this configuration
        */
        virtual bool init(DynamicsEngineConfigPtr config);

        void setMutex(boost::shared_ptr <boost::recursive_mutex> engineMutex);

        Eigen::Vector3f getGravity();

        virtual bool addObject(DynamicsObjectPtr o);
        virtual bool removeObject(DynamicsObjectPtr o);

        virtual bool addRobot(DynamicsRobotPtr r);
        virtual bool removeRobot(DynamicsRobotPtr r);

        virtual bool attachObjectToRobot(const std::string& robotName, const std::string& nodeName, DynamicsObjectPtr object);
        virtual bool attachObjectToRobot(DynamicsRobotPtr r, const std::string& nodeName, DynamicsObjectPtr object);

        virtual bool detachObjectFromRobot(const std::string& robotName, DynamicsObjectPtr object);
        virtual bool detachObjectFromRobot(DynamicsRobotPtr r, DynamicsObjectPtr object);


        std::vector<DynamicsRobotPtr> getRobots();

        /*!
            Returns all objects that are currently registered.
            Note, that all RobotNdoes are internally handled as DynamicObjects.
        */
        std::vector<DynamicsObjectPtr> getObjects();

        /*!
            Set floor. A standard floor object is created and added to the dynamics world.
        */
        virtual void createFloorPlane(const Eigen::Vector3f& pos, const Eigen::Vector3f& up);

        /*!
            By default all collisions are enabled.
            Here, collisions between two objects can be disabled.
        */
        virtual void disableCollision(DynamicsObject* o1, DynamicsObject* o2);
        virtual void enableCollision(DynamicsObject* o1, DynamicsObject* o2);
        //! removes all <collision disabled> settings for object o
        virtual void resetCollisions(DynamicsObject* o);

        /*!
            Disables collision check to all objects.
        */
        virtual void disableCollision(DynamicsObject* o1);
        /*!
            Enables collision check to all objects.
        */
        virtual void enableCollision(DynamicsObject* o1);


        /*!
            Check if two objects should be considered for collision detection.
        */
        virtual bool checkCollisionEnabled(DynamicsObject* o1, DynamicsObject* o2);
        /*!
            Check if object should be considered for collision detection.
        */
        virtual bool checkCollisionEnabled(DynamicsObject* o1);

        DynamicsObjectPtr getFloor()
        {
            return floor;
        }

        /*!
           Stores current floor description to floorPos and floorUp.
        */
        void getFloorInfo(Eigen::Vector3f& floorPos, Eigen::Vector3f& floorUp, double& floorExtendMM, double& floorDepthMM);

        struct DynamicsContactInfo
        {
            DynamicsObject* objectA;
            DynamicsObject* objectB;
            Eigen::Vector3f posGlobalA;
            Eigen::Vector3f posGlobalB;
            Eigen::Vector3f normalGlobalB;
            Eigen::Vector3f frictionDir1;
            Eigen::Vector3f frictionDir2;
            double distance;
            double combinedFriction;
            double combinedRestitution;
            double appliedImpulse;
        };

        virtual std::vector<DynamicsEngine::DynamicsContactInfo> getContacts();

        /*!
            Returns the dynamics version of r.
            An empty pointer is returned if no dynamic robot created from r has been added so far.
        */
        virtual DynamicsRobotPtr getRobot(VirtualRobot::RobotPtr r);
        virtual DynamicsRobotPtr getRobot(const std::string& robName);

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
        DynamicsEngineConfigPtr dynamicsConfig;

        std::vector<DynamicsObjectPtr> objects;
        std::vector<DynamicsRobotPtr> robots;

        std::map < DynamicsObject*, std::vector<DynamicsObject*> > collisionDisabled;
        std::vector<DynamicsObject*> collisionToAllDisabled;
        DynamicsObjectPtr floor;

        Eigen::Vector3f floorPos;
        Eigen::Vector3f floorUp;

        double floorExtendMM;
        double floorDepthMM;

        boost::shared_ptr <boost::recursive_mutex> engineMutexPtr;

    };

    typedef boost::shared_ptr<DynamicsEngine> DynamicsEnginePtr;


} // namespace SimDynamics

#endif // _SimDynamics_DynamicsEngine_h_
