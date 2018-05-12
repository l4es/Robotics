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
#ifndef _DYNAMICS_WORLD_H_
#define _DYNAMICS_WORLD_H_

#include "SimDynamics.h"
#include "DynamicsEngine/DynamicsEngine.h"
#include "DynamicsEngine/DynamicsObject.h"
#include "DynamicsEngine/DynamicsRobot.h"

namespace SimDynamics
{
    /*!
    */
    class SIMDYNAMICS_IMPORT_EXPORT DynamicsWorld
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Initialize the dynamics world singleton.
            If already created the instance is returned.
        */
        static DynamicsWorldPtr Init(DynamicsEngineConfigPtr config = DynamicsEngineConfigPtr());
        static DynamicsWorldPtr GetWorld();

        /*!
            Returns the engine implementation.
        */
        DynamicsEnginePtr getEngine();

        /*!
            Clean up and destroy the dynamic world singleton.
        */
        static void Close();
        virtual ~DynamicsWorld();

        /*!
            Build a dynamic version of your VirtualRobot::SceneObject. This can be an Obstacle or a ManipulationObject.
            Internally the value of o->getSimulationtype() is queried in order to specify which type of simulation should be performed.
            If the simualtion type is not specified a full dynamic object is created (as with eDynamic).
        */
        static DynamicsObjectPtr CreateDynamicsObject(VirtualRobot::SceneObjectPtr o);

        /*!
            Build a dynamic version of your VirtualRobot::Robot.
        */
        static DynamicsRobotPtr CreateDynamicsRobot(VirtualRobot::RobotPtr rob);

        /*!
            Add an object to the world.
        */
        bool addObject(DynamicsObjectPtr o);

        /*!
            Remove an object from the world.
        */
        bool removeObject(DynamicsObjectPtr o);

        /*!
            Add a robot to the world.
        */
        bool addRobot(DynamicsRobotPtr r);

        /*!
            Remove a robot from the world.
        */
        bool removeRobot(DynamicsRobotPtr r);

        std::vector<DynamicsRobotPtr> getRobots();

        /*!
            Create and setup floor.
            \param pos The displacement
            \param up The up vector. Currently supported: (+/-1,0,0), (0,+/-1,0) or (0,0,+/-1).
        */
        void createFloorPlane(const Eigen::Vector3f& pos = Eigen::Vector3f(0, 0, 0), const Eigen::Vector3f& up = Eigen::Vector3f(0, 0, 1.0f));

        /*!
            Since VirtualRobot usually uses MM and most physics engines like M, the models are automatically converted if this bool is set.
            You can disable this automatic feature by calling <DynamicsWorld::convertMM2M = false;> before starting with your application.
        */
        static bool convertMM2M; // standard: true;

    protected:
        // create world with static Init method.
        DynamicsWorld(DynamicsEngineConfigPtr config = DynamicsEngineConfigPtr());

        // see http://en.wikipedia.org/wiki/Singleton_pattern for details about correct implementations of singletons in C++
        friend class Cleanup;
        class Cleanup
        {
        public:
            ~Cleanup();
        };

        // The one and only world
        static DynamicsWorldPtr world;

        // The engine interface
        DynamicsEnginePtr engine;

    };

}

#endif /* _DYNAMICS_WORLD_H_ */
