/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2014 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_DynamicsRBDLTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Dynamics/dynamics.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <Eigen/Core>

using namespace VirtualRobot;

BOOST_AUTO_TEST_SUITE(DynamicsTests)

BOOST_AUTO_TEST_CASE(testRBDLConvertRobot)
{
    std::string robFile = "robots/ArmarIII/ArmarIII.xml";
    std::string rnsName = "LeftArm";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(robFile);
    BOOST_REQUIRE(fileOK);
    RobotPtr robot;
    BOOST_REQUIRE_NO_THROW(robot = RobotIO::loadRobot(robFile));
    BOOST_REQUIRE(robot);
    BOOST_REQUIRE(robot->hasRobotNodeSet(rnsName));
    RobotNodeSetPtr rns = robot->getRobotNodeSet(rnsName);

    Dynamics dynamics(rns);


}

BOOST_AUTO_TEST_SUITE_END()
