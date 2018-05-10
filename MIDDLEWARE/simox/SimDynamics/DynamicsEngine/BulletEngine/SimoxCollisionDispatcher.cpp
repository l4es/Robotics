
#include "SimoxCollisionDispatcher.h"
#include "BulletEngine.h"


using namespace VirtualRobot;

namespace SimDynamics
{


    SimoxCollisionDispatcher::SimoxCollisionDispatcher(BulletEngine* engine, btCollisionConfiguration* collisionConfiguration)
        : engine(engine), btCollisionDispatcher(collisionConfiguration)
    {
    }

    SimoxCollisionDispatcher::~SimoxCollisionDispatcher()
    {

    }
    bool SimoxCollisionDispatcher::needsCollision(/*const*/ btCollisionObject* body0,/*const*/ btCollisionObject* body1)
    {
        SimDynamics::BulletObject* o0 = static_cast<SimDynamics::BulletObject*>(body0->getUserPointer());
        SimDynamics::BulletObject* o1 = static_cast<SimDynamics::BulletObject*>(body1->getUserPointer());

        if (!engine || engine->checkCollisionEnabled(o0, o1))
        {
            return btCollisionDispatcher::needsCollision(body0, body1);
        }

        return false;
    }

    bool SimoxCollisionDispatcher::needsResponse(/*const*/ btCollisionObject* body0,/*const*/ btCollisionObject* body1)
    {
        SimDynamics::BulletObject* o0 = static_cast<SimDynamics::BulletObject*>(body0->getUserPointer());
        SimDynamics::BulletObject* o1 = static_cast<SimDynamics::BulletObject*>(body1->getUserPointer());

        if (!engine || engine->checkCollisionEnabled(o0, o1))
        {
            return btCollisionDispatcher::needsResponse(body0, body1);
        }

        return false;
    }


}
