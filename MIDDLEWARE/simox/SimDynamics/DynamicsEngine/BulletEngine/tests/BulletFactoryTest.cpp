/**
* @package    SimDynamics
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE SimDynamics_BulletFactoryTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <SimDynamics/DynamicsWorld.h>
#include <SimDynamics/DynamicsEngine/BulletEngine/BulletEngineFactory.h>

#include <VirtualRobot/Obstacle.h>

#include <string>


BOOST_AUTO_TEST_CASE(testSimDynamicsBulletFactoryCreateWorld)
{
    SimDynamics::DynamicsWorldPtr world;
    BOOST_REQUIRE_NO_THROW(world = SimDynamics::DynamicsWorld::Init());
    BOOST_REQUIRE(world);
    BOOST_REQUIRE_NO_THROW(SimDynamics::DynamicsWorld::Close());
}

BOOST_AUTO_TEST_CASE(testSimDynamicsBulletFactoryCreateObject)
{
    SimDynamics::DynamicsWorldPtr world;
    BOOST_REQUIRE_NO_THROW(world = SimDynamics::DynamicsWorld::Init());
    BOOST_REQUIRE(world);


    VirtualRobot::ObstaclePtr o;
    BOOST_REQUIRE_NO_THROW(o = VirtualRobot::Obstacle::createBox(1000.0f, 1000.0f, 1000.0f));
    BOOST_REQUIRE(o);
    o->setMass(1.0f); // 1kg

    SimDynamics::DynamicsObjectPtr dynObj;
    BOOST_REQUIRE_NO_THROW(dynObj = world->CreateDynamicsObject(o));
    BOOST_REQUIRE(dynObj);

    bool ok;

    BOOST_REQUIRE_NO_THROW(ok = world->addObject(dynObj));
    BOOST_REQUIRE(ok);

    BOOST_REQUIRE_NO_THROW(ok = world->removeObject(dynObj));
    BOOST_REQUIRE(ok);

    // clean up to avoid memory leaks
    dynObj.reset();
    o.reset();
    // no VirtualRobot objects (i.e. visualizations) should be alive when cleaning up, since the visualization framework shuts down
    // and VirtualRobot visualizations will fail when trying to unregister in their destructor.
    VirtualRobot::RuntimeEnvironment::cleanup();

    BOOST_REQUIRE_NO_THROW(SimDynamics::DynamicsWorld::Close());
}
