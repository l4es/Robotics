/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotSensorTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Nodes/Sensor.h>
#include <VirtualRobot/Nodes/PositionSensor.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_SUITE(Sensor)

BOOST_AUTO_TEST_CASE(testPositionSensor)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Joint type='prismatic'>"
        "    <TranslationDirection x='0' y='0' z='1'/>"
        "    <Limits units='mm' lo='0' hi='1000'/>"
        "  </Joint>"
        "  <Sensor type='position' name='sensor1'>"
        "    <Transform>"
        "       <Translation x='100' y='50' z='0'/>"
        "    </Transfrom>"
        "  </Sensor>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);
    VirtualRobot::RobotNodePtr rn = rob->getRobotNode("Joint1");
    BOOST_REQUIRE(rn);
    BOOST_REQUIRE(rn->hasSensor("sensor1"));

    VirtualRobot::PositionSensorPtr ps = boost::dynamic_pointer_cast<VirtualRobot::PositionSensor>(rn->getSensor("sensor1"));
    BOOST_REQUIRE(ps);

    Eigen::Matrix4f p = ps->getGlobalPose();
    Eigen::Matrix4f p2 = Eigen::Matrix4f::Identity();
    p2.block(0, 3, 3, 1) << 100.0f, 50.0f, 0;

    BOOST_REQUIRE(p.isApprox(p2));

    rn->setJointValue(333.0f);
    p = ps->getGlobalPose();
    p2.block(0, 3, 3, 1) << 100.0f, 50.0f, 333.0f;

    BOOST_REQUIRE(p.isApprox(p2));

}

BOOST_AUTO_TEST_SUITE_END()
