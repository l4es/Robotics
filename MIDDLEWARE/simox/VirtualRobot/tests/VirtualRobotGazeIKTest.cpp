/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2014 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotGazeIKTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/IK/GazeIK.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodePrismatic.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

BOOST_AUTO_TEST_SUITE(GazeIK)

BOOST_AUTO_TEST_CASE(testGazeIK)
{
    const std::string robotString =
        "<?xml version='1.0' encoding='UTF-8' ?>                                                                      "
        "                                                                                                             "
        "<Robot Type='ArmarIII Head' RootNode='Head Base'>                                                            "
        "                                                                                                             "
        "    <RobotNode name='Head Base'>                                                                             "
        "        <Transform>                                                                                          "
        "            <DH theta='90' d='0' a='0' alpha='0' units='degree'/>                                            "
        "        </Transform>                                                                                         "
        "        <Child name='Neck_1_Pitch'/>                                                                         "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='Neck_1_Pitch'>                                                                          "
        "        <Transform>                                                                                          "
        "            <DH theta='0' d='0' a='0' alpha='90' units='degree'/>                                            "
        "            <DH theta='90' d='0' a='0' alpha='0' units='degree'/>                                            "
        "        </Transform>                                                                                         "
        "        <Joint type='revolute'>                                                                              "
        "            <axis x='0' y='0' z='1'/>                                                                        "
        "            <Limits unit='degree' lo='-45' hi='45'/>                                                         "
        "        </Joint>                                                                                             "
        "        <Child name='Neck_2_Roll'/>                                                                          "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='Neck_2_Roll'>                                                                           "
        "        <Transform>                                                                                          "
        "			<DH a='0' d='0' theta='0' alpha='90' units='degree'/>                                             "
        "			<DH a='0' d='0' theta='90' alpha='0' units='degree'/>                                             "
        "		 </Transform>                                                                                         "
        "        <Joint type='revolute'>                                                                              "
        "            <axis x='0' y='0' z='1'/>                                                                        "
        "            <Limits unit='degree' lo='-45' hi='45'/>                                                         "
        "        </Joint>                                                                                             "
        "        <Child name='Neck_3_Yaw'/>                                                                           "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='Neck_3_Yaw'>                                                                            "
        "       <Transform>                                                                                           "
        "			<DH a='0' d='0' theta='0' alpha='90' units='degree'/>                                             "
        "			<DH a='0' d='0' theta='90' alpha='0' units='degree'/>                                             "
        "		 </Transform>                                                                                         "
        "        <Joint type='revolute'>                                                                              "
        "            <axis x='0' y='0' z='1'/>                                                                        "
        "            <Limits unit='degree' lo='-45' hi='45'/>                                                         "
        "        </Joint>                                                                                             "
        "        <Child name='Head_Tilt'/>                                                                            "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='Head_Tilt'>                                                                             "
        "        <Transform>                                                                                          "
        "			<DH theta='0' d='120' a='0' alpha='90' units='degree'/>                                           "
        "		 </Transform>                                                                                         "
        "        <Joint type='revolute'>                                                                              "
        "            <axis x='0' y='0' z='1'/>                                                                        "
        "            <Limits unit='degree' lo='-45' hi='45'/>                                                         "
        "        </Joint>                                                                                             "
        "        <Child name='Head Center1'/>                                                                         "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='Head Center1'>                                                                          "
        "        <Transform>                                                                                          "
        "			<DH theta='0' d='0' a='0' alpha='-90' units='degree'/>                                            "
        "		 </Transform>                                                                                         "
        "        <Child name='Head Center'/>                                                                          "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='Head Center'>                                                                           "
        "        <Transform>                                                                                          "
        "			<DH theta='0' d='54.5' a='0' alpha='-90' units='degree'/>                                         "
        "		 </Transform>                                                                                         "
        "        <Child name='Cameras'/>                                                                              "
        "        <Child name='Jaw'/>                                                                                  "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='Jaw'>                                                                                   "
        "        <Joint type='revolute'>                                                                              "
        "            <axis x='0' y='0' z='1'/>                                                                        "
        "            <Limits unit='degree' lo='-30' hi='30'/>                                                         "
        "        </Joint>                                                                                             "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='Cameras'>                                                                               "
        "        <Transform>                                                                                          "
        "            <Translation x='100' y='0' z='0'/>                                                               "
        "        </Transform>                                                                                         "
        "        <Joint type='revolute'>                                                                              "
        "            <Axis x='0' y='0' z='1'/>                                                                        "
        "            <Limits unit='degree' lo='-30' hi='45'/>                                                         "
        "        </Joint>                                                                                             "
        "        <Child name='VirtualCentralGaze'/>                                                                   "
        "    </RobotNode>                                                                                             "
        "                                                                                                             "
        "    <RobotNode name='VirtualCentralGaze'>                                                                    "
        "        <Joint type='prismatic'>                                                                             "
        "            <TranslationDirection x='1' y='0' z='0'/>                                                        "
        "            <Limits unit='mm' lo='0' hi='10000'/>                                                            "
        "        </Joint>                                                                                             "
        "	</RobotNode>                                                                                              "
        "                                                                                                             "
        "    <RobotNodeSet name='IKVirtualGaze' kinematicRoot='Neck_1_Pitch' tcp='VirtualCentralGaze'>                "
        "        <Node name='Neck_1_Pitch'/>                                                                          "
        "        <Node name='Neck_2_Roll'/>                                                                           "
        "        <Node name='Neck_3_Yaw'/>                                                                            "
        "        <Node name='Head_Tilt'/>                                                                             "
        "        <Node name='Cameras'/>                                                                               "
        "        <Node name='VirtualCentralGaze'/>                                                                    "
        "    </RobotNodeSet>                                                                                          "
        "                                                                                                             "
        "</Robot>                                                                                                     ";

    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    const std::string rnsName = "IKVirtualGaze";
    const std::string nodeTransName = "VirtualCentralGaze";
    VirtualRobot::RobotNodeSetPtr rns = rob->getRobotNodeSet(rnsName);
    VirtualRobot::RobotNodePtr node = rob->getRobotNode(nodeTransName);
    VirtualRobot::RobotNodePrismaticPtr nodeTrans = boost::dynamic_pointer_cast<VirtualRobot::RobotNodePrismatic>(node);

    BOOST_REQUIRE(rns);
    BOOST_REQUIRE(nodeTrans);

    VirtualRobot::GazeIK ik(rns, nodeTrans);

    // look to front
    Eigen::Vector3f goal1;
    goal1 = nodeTrans->getGlobalPose().block(0, 3, 3, 1); // current gaze point
    goal1(1) += 1000.0f;

    // look to front/right
    Eigen::Vector3f goal2;
    goal2 = nodeTrans->getGlobalPose().block(0, 3, 3, 1); // current gaze point
    goal2(1) += 1000.0f;
    goal2(0) += 300.0f;

    // look to front/left
    Eigen::Vector3f goal3;
    goal3 = nodeTrans->getGlobalPose().block(0, 3, 3, 1); // current gaze point
    goal3(1) += 1000.0f;
    goal3(0) -= 300.0f;

    Eigen::VectorXf jvZero(6);
    jvZero.setZero();

    bool ok1 = ik.solve(goal1);
    BOOST_REQUIRE(ok1);
    float diff1 = (nodeTrans->getGlobalPose().block(0, 3, 3, 1) - goal1).norm();
    BOOST_CHECK_LT(diff1, ik.getMaxPosError());

    rns->setJointValues(jvZero);
    ik.enableJointLimitAvoidance(true);
    bool ok2 = ik.solve(goal2);
    BOOST_REQUIRE(ok2);
    float diff2 = (nodeTrans->getGlobalPose().block(0, 3, 3, 1) - goal2).norm();
    BOOST_CHECK_LT(diff2, ik.getMaxPosError());

    rns->setJointValues(jvZero);
    ik.enableJointLimitAvoidance(false);
    bool ok3 = ik.solve(goal3);
    BOOST_REQUIRE(ok3);
    float diff3 = (nodeTrans->getGlobalPose().block(0, 3, 3, 1) - goal3).norm();
    BOOST_CHECK_LT(diff3, ik.getMaxPosError());

}

BOOST_AUTO_TEST_SUITE_END()
