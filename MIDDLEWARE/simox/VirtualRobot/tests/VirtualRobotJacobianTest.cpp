/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotJacobianTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/DifferentialIK.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

BOOST_AUTO_TEST_SUITE(RobotNode)

#define MAX_ERROR 0.3f
#define STEP_SIZE 0.001f

BOOST_AUTO_TEST_CASE(testJacobianRevoluteJoint)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"

        "  <Joint type='revolute'>"
        "   <Limits unit='degree' lo='-45' hi='45'/>"
        "	<Axis x='1' y='0' z='0'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"

        " <RobotNode name='Joint2'>"
        "  <Transform>"
        "     <Translation x='100' y='200' z='0'/>"
        "  </Transform>"
        "  <Joint type='revolute'>"
        "   <Limits unit='degree' lo='-45' hi='45'/>"
        "	<Axis x='0' y='0' z='1'/>"
        "  </Joint>"
        "  <Child name='Joint3'/>"
        " </RobotNode>"

        " <RobotNode name='Joint3'>"
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
    const std::string node3 = "Joint3";
    VirtualRobot::RobotNodePtr r1 = rob->getRobotNode(node1);
    VirtualRobot::RobotNodePtr r2 = rob->getRobotNode(node2);
    VirtualRobot::RobotNodePtr r3 = rob->getRobotNode(node3);
    BOOST_REQUIRE(r1);
    BOOST_REQUIRE(r2);
    BOOST_REQUIRE(r3);

    std::vector< VirtualRobot::RobotNodePtr > nodes;
    nodes.push_back(r1);
    nodes.push_back(r2);
    nodes.push_back(r3);
    VirtualRobot::RobotNodeSetPtr kc(VirtualRobot::RobotNodeSet::createRobotNodeSet(rob, "KinChain", nodes, r1));
    BOOST_REQUIRE(kc);
    BOOST_CHECK_EQUAL(kc->isKinematicChain(), true);

    VirtualRobot::RobotNodeSetPtr node_set;

    VirtualRobot::DifferentialIK ik(kc);
    Eigen::VectorXf jV(3);
    jV << 0.78f, 0.78f, 0;
    rob->setJointValues(kc, jV);

    // Calculate the Jacobi matrix at the given position
    Eigen::MatrixXf jacobian = ik.getJacobianMatrix(kc->getTCP());

    // Calculate the Differences quotient
    Eigen::Matrix4f a = r3->getGlobalPose();
    Eigen::MatrixXf DiffQuot(3, 2);
    jV << 0.78f + STEP_SIZE, 0.78f, 0 ;
    rob->setJointValues(kc, jV);
    DiffQuot.block<3, 1>(0, 0) = (r3->getGlobalPose().block<3, 1>(0, 3) - a.block<3, 1>(0, 3)) / STEP_SIZE;
    jV << 0.78f, 0.78f + STEP_SIZE, 0;
    rob->setJointValues(kc, jV);
    DiffQuot.block<3, 1>(0, 1) = (r3->getGlobalPose().block<3, 1>(0, 3) - a.block<3, 1>(0, 3)) / STEP_SIZE;

    // Compare both and check if they are similar enough.

    //std::cout << "Jacobian:\n " << jacobian.block<3,2>(0,0) << std::endl;
    //std::cout << "Differential quotient:\n " << DiffQuot << std::endl;
    //std::cout << (  (jacobian.block<3,2>(0,0) -  DiffQuot).array().abs() < 0.2     ).all() << std::endl;
    BOOST_CHECK(((jacobian.block<3, 2>(0, 0) -  DiffQuot).array().abs() < MAX_ERROR).all());

}

BOOST_AUTO_TEST_SUITE_END()
