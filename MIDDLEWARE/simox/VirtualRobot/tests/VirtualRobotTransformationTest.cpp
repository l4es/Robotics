/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotTransformationTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

BOOST_AUTO_TEST_SUITE(RobotNode)

#define FLOAT_CLOSE_TO_DIFF 1e-7f

inline void CHECK_TRANSFORMATION_MATRIX(Eigen::Matrix4f& m, float x, float y, float z)
{
    // first row
    BOOST_CHECK_CLOSE(1.0f, m(0, 0), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, m(0, 1), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, m(0, 2), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(x, m(0, 3), FLOAT_CLOSE_TO_DIFF);
    // second row
    BOOST_CHECK_CLOSE(0.0f, m(1, 0), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(1.0f, m(1, 1), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, m(1, 2), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(y, m(1, 3), FLOAT_CLOSE_TO_DIFF);
    // third row
    BOOST_CHECK_CLOSE(0.0f, m(2, 0), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, m(2, 1), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(1.0f, m(2, 2), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(z, m(2, 3), FLOAT_CLOSE_TO_DIFF);
    // fourth row
    BOOST_CHECK_CLOSE(0.0f, m(3, 0), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, m(3, 1), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(0.0f, m(3, 2), FLOAT_CLOSE_TO_DIFF);
    BOOST_CHECK_CLOSE(1.0f, m(3, 3), FLOAT_CLOSE_TO_DIFF);
}

BOOST_AUTO_TEST_CASE(testRobotNodePrismaticTransformation)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "    <Transform>"
        "     <Translation x='100' y='0' z='0'/>"
        "    </Transform>"
        "  <Joint type='prismatic'>"
        "   <Limits unit='mm' lo='-300' hi='500'/>"
        "   <TranslationDirection x='0' y='0' z='1'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "    <Transform>"
        "     <Translation x='0' y='200' z='0'/>"
        "    </Transform>"
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
    Eigen::Matrix4f m1 = r1->getGlobalPose();
    Eigen::Matrix4f m2 = r2->getGlobalPose();

    CHECK_TRANSFORMATION_MATRIX(m1, 100.0f, 0, 0);
    CHECK_TRANSFORMATION_MATRIX(m2, 100.0f, 200.0f, 0);

    // play around with joint values
    rob->setJointValue(r1, 150.0f);

    m1 = r1->getGlobalPose();
    m2 = r2->getGlobalPose();

    CHECK_TRANSFORMATION_MATRIX(m1, 100.0f, 0, 150.0f);
    CHECK_TRANSFORMATION_MATRIX(m2, 100.0f, 200.0f, 150.0f);

    // play around with joint values
    rob->setJointValue(r1, -5000.0f);

    m1 = r1->getGlobalPose();
    m2 = r2->getGlobalPose();

    CHECK_TRANSFORMATION_MATRIX(m1, 100.0f, 0, -300.0f);
    CHECK_TRANSFORMATION_MATRIX(m2, 100.0f, 200.0f, -300.0f);

}

BOOST_AUTO_TEST_SUITE_END()
