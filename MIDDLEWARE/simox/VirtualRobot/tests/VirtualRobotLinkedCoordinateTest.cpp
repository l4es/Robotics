/**
* @package    VirtualRobot
* @author     Stefan Ulbrich, Nikolaus Vahrenkamp
* @copyright  2010,2011 Stefan Ulbrich, Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_CoordinatesTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/LinkedCoordinate.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

BOOST_AUTO_TEST_SUITE(RobotNode)

#define FLOAT_CLOSE_TO_DIFF 1e-7f


BOOST_AUTO_TEST_CASE(testIntelligentCoordinate)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"

        " <RobotNode name='Joint1'>"
        "  <Transform>"
        "     <Translation x='100' y='0' z='0'/>"
        "  </Transform>"
        "  <Joint type='revolute'>"
        "   <Limits unit='degree' lo='-45' hi='45'/>"
        "	<Axis x='1' y='0' z='0'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"

        " <RobotNode name='Joint2'>"
        "  <Transform>"
        "     <Translation x='0' y='200' z='0'/>"
        "  </Transform>"
        "  <Joint type='revolute'>"
        "   <Limits unit='degree' lo='-45' hi='45'/>"
        "	<Axis x='0' y='0' z='1'/>"
        "  </Joint>"
        " </RobotNode>"

        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node1 = "Joint1";
    const std::string node2 = "Joint2";
    VirtualRobot::RobotNodePtr r1 = rob->getRobotNode(node1);
    VirtualRobot::RobotNodePtr r2 = rob->getRobotNode(node2);
    BOOST_REQUIRE(r1);
    BOOST_REQUIRE(r2);

    BOOST_REQUIRE_NO_THROW(VirtualRobot::LinkedCoordinate coord(rob));

    Eigen::Vector3f origin;
    origin << 0, 0, 0;

    VirtualRobot::LinkedCoordinate coord(rob);

    BOOST_REQUIRE_THROW(coord.set("Joint4", origin), VirtualRobot::VirtualRobotException);
    BOOST_REQUIRE_THROW(coord.set(VirtualRobot::RobotNodePtr()), VirtualRobot::VirtualRobotException);

    BOOST_REQUIRE_NO_THROW(coord.set(r2));
    BOOST_REQUIRE_NO_THROW(coord.changeFrame(r1));
    BOOST_REQUIRE_THROW(coord.changeFrame("Joint4"), VirtualRobot::VirtualRobotException);

    Eigen::Vector3f translation; // the translation from the origin of r2 to r1.

    translation = coord.getPosition();
    BOOST_CHECK_CLOSE(0.0f, translation(0), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(200.0f, translation(1), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, translation(2), FLOAT_CLOSE_TO_DIFF);

    Eigen::Vector3f translation2 = coord.getInFrame(r2).block<3, 1>(0, 3);
    BOOST_CHECK_CLOSE(0.0f, translation2(0), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, translation2(1), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, translation2(2), FLOAT_CLOSE_TO_DIFF);

    // design more advanced and complete tests
}

BOOST_AUTO_TEST_SUITE_END()
