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
#ifndef _SimDynamics_SimoxCollisionDispatcher_h_
#define _SimDynamics_SimoxCollisionDispatcher_h_

#include "../../SimDynamics.h"
#include "BulletEngine.h"

#include <btBulletCollisionCommon.h>

#include <VirtualRobot/Nodes/RobotNodeActuator.h>
#include <VirtualRobot/SceneObject.h>

#include <vector>

namespace SimDynamics
{

    /*!
        Handle disabled objects
    */
    struct SIMDYNAMICS_IMPORT_EXPORT SimoxCollisionDispatcher : public btCollisionDispatcher
    {
        SimoxCollisionDispatcher(BulletEngine* engine, btCollisionConfiguration* collisionConfiguration);
        virtual ~SimoxCollisionDispatcher();
        virtual bool    needsCollision(/*const*/ btCollisionObject* body0,/*const*/ btCollisionObject* body1);
        virtual bool    needsResponse(/*const*/ btCollisionObject* body0,/*const*/ btCollisionObject* body1);

    protected:
        BulletEngine* engine;
    };
}


#endif // _SimDynamics_SimoxCollisionDispatcher_h_

