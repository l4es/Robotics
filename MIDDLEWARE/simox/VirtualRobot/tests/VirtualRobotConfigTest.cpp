/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotConfigTest

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_SUITE(Scene)

BOOST_AUTO_TEST_CASE(testRobotConfigInvalidCreation)
{
    BOOST_REQUIRE_THROW(VirtualRobot::RobotConfigPtr c(new VirtualRobot::RobotConfig(VirtualRobot::RobotPtr(), "")), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotConfigSetConfig)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    VirtualRobot::RobotConfigPtr c;
    BOOST_REQUIRE_NO_THROW(c.reset(new VirtualRobot::RobotConfig(rob, "test")));
    BOOST_REQUIRE(c);

    const std::string node1 = "Joint1";
    BOOST_REQUIRE_NO_THROW(c->setConfig(node1, 0.0f));
}



BOOST_AUTO_TEST_CASE(testRobotConfigSetInvalidConfig)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    VirtualRobot::RobotConfigPtr c;
    BOOST_REQUIRE_NO_THROW(c.reset(new VirtualRobot::RobotConfig(rob, "test")));
    BOOST_REQUIRE(c);

    const std::string node2 = "JointNotPresent";
    bool ok = c->setConfig(node2, 0.0f);
    BOOST_REQUIRE(!ok);
}



BOOST_AUTO_TEST_SUITE_END()
