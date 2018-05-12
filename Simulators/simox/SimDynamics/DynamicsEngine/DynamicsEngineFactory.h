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
#ifndef _SimDynamics_DynamicsEngineFactory_h_
#define _SimDynamics_DynamicsEngineFactory_h_

#include "../SimDynamics.h"
#include "DynamicsEngine.h"
#include "DynamicsRobot.h"
#include <VirtualRobot/AbstractFactoryMethod.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/SceneObject.h>

namespace SimDynamics
{

    /*!
        An interface for a physics engine factory.
        Currently, the first registered factory is used to create the physics engine abstraction, so multiple coexisting factories are not yet supported.
        @see BulletEngineFactory
    */
    class SIMDYNAMICS_IMPORT_EXPORT DynamicsEngineFactory  : public AbstractFactoryMethod<DynamicsEngineFactory, void*>
    {
    public:

        DynamicsEngineFactory()
        {
            ;
        }
        virtual ~DynamicsEngineFactory()
        {
            ;
        }

        //! Derived classes must return the correct engine implementation.
        virtual DynamicsEnginePtr createEngine(DynamicsEngineConfigPtr config = DynamicsEngineConfigPtr())
        {
            return DynamicsEnginePtr();
        }

        virtual DynamicsObjectPtr createObject(VirtualRobot::SceneObjectPtr o)
        {
            return DynamicsObjectPtr(new DynamicsObject(o));
        }
        virtual DynamicsRobotPtr createRobot(VirtualRobot::RobotPtr robot)
        {
            return DynamicsRobotPtr(new DynamicsRobot(robot));
        }

    };

    typedef boost::shared_ptr<DynamicsEngineFactory> DynamicsEngineFactoryPtr;

} // namespace SimDynamics

#endif // _SimDynamics_DynamicsEngineFactory_h_
