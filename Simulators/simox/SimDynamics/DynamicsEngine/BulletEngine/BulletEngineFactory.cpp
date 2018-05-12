
#include "BulletEngineFactory.h"
#include "BulletObject.h"
#include "BulletRobot.h"

namespace SimDynamics
{

    BulletEngineFactory::BulletEngineFactory()
    {
    }


    BulletEngineFactory::~BulletEngineFactory()
    {
    }


    /**
     * register this class in the super class factory
     */
    DynamicsEngineFactory::SubClassRegistry BulletEngineFactory::registry(BulletEngineFactory::getName(), &BulletEngineFactory::createInstance);


    /**
     * \return "bullet"
     */
    std::string BulletEngineFactory::getName()
    {
        return "bullet";
    }


    /**
     * \return new instance of BulletEngineFactory
     */
    boost::shared_ptr<DynamicsEngineFactory> BulletEngineFactory::createInstance(void*)
    {
        boost::shared_ptr<BulletEngineFactory> bulletFactory(new BulletEngineFactory());
        return bulletFactory;
    }

    DynamicsEnginePtr BulletEngineFactory::createEngine(DynamicsEngineConfigPtr config)
    {
        BulletEnginePtr bulletEngine(new BulletEngine());
        //DynamicsEngine::DynamicsWorldInfo i; // standard gravity
        bulletEngine->init(config);
        return bulletEngine;
    }

    DynamicsObjectPtr BulletEngineFactory::createObject(VirtualRobot::SceneObjectPtr o)
    {
        return BulletObjectPtr(new BulletObject(o));
    }

    SimDynamics::DynamicsRobotPtr BulletEngineFactory::createRobot(VirtualRobot::RobotPtr robot)
    {
        return BulletRobotPtr(new BulletRobot(robot));
    }

} // namespace SimDynamics
