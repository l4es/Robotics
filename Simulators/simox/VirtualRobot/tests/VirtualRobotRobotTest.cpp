/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotRobotTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/Sensor.h>
#include <VirtualRobot/Nodes/PositionSensor.h>
#include <string>

BOOST_AUTO_TEST_SUITE(RobotFactory)

BOOST_AUTO_TEST_CASE(testRobotFactoryEmptyXML)
{
    const std::string robotString = "";
    BOOST_REQUIRE_THROW((VirtualRobot::RobotIO::createRobotFromString(robotString)), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryUnclosedRobotTag)
{
    const std::string robotString = "<Robot>";
    VirtualRobot::RobotPtr robot;
    BOOST_REQUIRE_THROW(robot = VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryOnlyClosedRobotTag)
{
    const std::string robotString = "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryEmptyRobotTag)
{
    const std::string robotString = "<Robot></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryEmptyTypeString)
{
    const std::string robotString = "<Robot Type=''></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryNotExistentType)
{
    const std::string robotString = "<Robot Type='XYZ'></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotRobotMacro)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node = "Joint1";
    VirtualRobot::RobotNodePtr r1 = rob->getRobotNode(node);
    BOOST_REQUIRE(r1);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryEmptyRootNodeString)
{
    const std::string robotString = "<Robot RootNode=''></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testRobotFactoryNotExistentRootNode)
{
    const std::string robotString = "<Robot RootNode='JointX'></Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotValidEndeffector)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint3'/>"
        " </RobotNode>"
        " <RobotNode name='Joint3'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint4'/>"
        " </RobotNode>"
        " <RobotNode name='Joint4'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        " </RobotNode>"
        " <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
        "  <Static>"
        "   <Node name='Joint1'/>"
        "   <Node name='Joint2'/>"
        "  </Static>"
        "  <Actor name='actor1'>"
        "   <Node name='Joint3'/>"
        "   <Node name='Joint4'/>"
        "  </Actor>"
        " </Endeffector>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotInvariantTagPosition)
{
    // test if references to nodes are resolved correctly if the
    // nodes are defined after they are referenced
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <RobotNode name='Joint3'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint4'/>"
        " </RobotNode>"
        " <RobotNode name='Joint4'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        " </RobotNode>"
        " <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
        "  <Static>"
        "   <Node name='Joint1'/>"
        "   <Node name='Joint2'/>"
        "  </Static>"
        "  <Actor name='actor1'>"
        "   <Node name='Joint3'/>"
        "   <Node name='Joint4'/>"
        "  </Actor>"
        " </Endeffector>"
        " <RobotNode name='Joint1'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Visualization enable='true'>"
        "  </Visualization>"
        "  <Child name='Joint3'/>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorWrongChildTag)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Visualization enable='true'>"
        "   <CoordinateAxis enable='true' scaling='1' text='Axis1'/>"
        "  </Visualization>"
        " </RobotNode>"
        " <Endeffector name='endeffector1' base='Joint1' tcp='Joint1'>"
        "  <XYZ>"
        "  </XYZ>"
        "  <Static>"
        "   <Node name='Joint1'/>"
        "  </Static>"
        " </Endeffector>"
        "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), std::exception);//VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorWithoutNameTag)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <Endeffector base='Joint1' tcp='Joint1'>"
        " </Endeffector>"
        "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorMissingBasenodeTag)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <Endeffector name ='e1'>"
        " </Endeffector>"
        "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotEndeffectorMissingBasenode)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' StandardName='TestRobot' RootNode='Joint1'>"
        " <Endeffector name ='e1' base='Joint1' tcp='Joint1'>"
        " </Endeffector>"
        "</Robot>";
    BOOST_REQUIRE_THROW(VirtualRobot::RobotIO::createRobotFromString(robotString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotPhysicsTag)
{
    const std::string robotString =
        "<Robot Type='DemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Physics>"
        "   <Mass value='100' units='kg'/>"
        "   <CoM location='joint' x='10' y='20' z='30' units='mm'/>"
        "   <InertiaMatrix unitsWeight='ton' unitsLength='mm'>"
        "     <row1 c1='1' c2='2' c3='3'/>"
        "     <row2 c1='4' c2='5' c3='6'/>"
        "     <row3 c1='7' c2='8' c3='9'/>"
        "   </InertiaMatrix>"
        "  </Physics>"
        "  <Transform>"
        "    <DH a='1' d='0' theta='0' alpha='0' units='degree' unitsLength='m'/>"
        "  </Transform>"
        "  <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <MaxVelocity value='36' unitsLength='mm' unitsTime='h'/>"
        "    <MaxAcceleration value='36' unitsTime='min'/>"
        "    <MaxTorque value='0.2' units='meter'/>"
        "  </Joint>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);
    VirtualRobot::RobotNodePtr rn = rob->getRobotNode("Joint1");
    BOOST_REQUIRE(rn);
    float mass = rn->getMass();
    BOOST_CHECK_EQUAL(mass, 100.0f);
    float vel = rn->getMaxVelocity();
    BOOST_CHECK_CLOSE(vel, 1e-5f, 0.01f);
    float acc = rn->getMaxAcceleration();
    BOOST_CHECK_CLOSE(acc, 0.01f, 0.01f);
    float to = rn->getMaxTorque();
    BOOST_CHECK_CLOSE(to, 0.2f, 0.01f);
    Eigen::Vector3f com = rn->getCoMLocal();
    bool comOK = com.isApprox(Eigen::Vector3f(10.0f, 20.0f, 30.0f));
    BOOST_REQUIRE(comOK);

    Eigen::Matrix3f inertia = rn->getInertiaMatrix();
    Eigen::Matrix3f expectedMat;
    expectedMat << 0.001f, 0.002f, 0.003f, 0.004f, 0.005f, 0.006f, 0.007f, 0.008f, 0.009f;
    bool inertiaMatrixOK = inertia.isApprox(expectedMat);
    BOOST_REQUIRE(inertiaMatrixOK);
    Eigen::Matrix4f m = rn->getLocalTransformation();
    BOOST_CHECK_EQUAL(m(0, 3), 1000.0f);
}


BOOST_AUTO_TEST_CASE(testVirtualRobotDependendNodes)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Transform>"
        "    <DH a='1' d='0' theta='0' alpha='-90' units='degree' unitsLength='m'/>"
        "  </Transform>"
        "  <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='0' hi='180'/>"
        "    <PropagateJointValue factor='0.5' name='Joint2'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Transform>"
        "    <DH a='0' d='0' theta='0' alpha='0' units='degree'/>"
        "  </Transform>"
        "   <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='0' hi='90'/>"
        "   </Joint>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node1 = "Joint1";
    const std::string node2 = "Joint2";
    VirtualRobot::RobotNodePtr r1 = rob->getRobotNode(node1);
    BOOST_REQUIRE(r1);
    VirtualRobot::RobotNodePtr r2 = rob->getRobotNode(node2);
    BOOST_REQUIRE(r2);
    float j1, j2;
    r1->setJointValue(0.2f);
    j1 = r1->getJointValue();
    j2 = r2->getJointValue();
    BOOST_CHECK_EQUAL(j1, 0.2f);
    BOOST_CHECK_EQUAL(j2, 0.1f);
    r1->setJointValue(float(M_PI));
    j1 = r1->getJointValue();
    j2 = r2->getJointValue();
    BOOST_CHECK_CLOSE(j1, float(M_PI), 0.1f);
    BOOST_CHECK_CLOSE(j2, float(M_PI / 2.0), 0.1f);

    // disable propagate feature
    r1->propagateJointValue("Joint2", 0.0f);
    r2->setJointValue(0.5f);
    r1->setJointValue(0.2f);
    j1 = r1->getJointValue();
    j2 = r2->getJointValue();
    BOOST_CHECK_CLOSE(j1, 0.2f, 0.1f);
    BOOST_CHECK_CLOSE(j2, 0.5f, 0.1f);

}


BOOST_AUTO_TEST_CASE(testVirtualRobotToXML)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "  <Physics>"
        "   <Mass value='100' units='kg'/>"
        "   <CoM location='joint' x='10' y='20' z='30' units='mm'/>"
        "   <InertiaMatrix unitsWeight='ton' unitsLength='mm'>"
        "     <row1 c1='1' c2='2' c3='3'/>"
        "     <row2 c1='4' c2='5' c3='6'/>"
        "     <row3 c1='7' c2='8' c3='9'/>"
        "   </InertiaMatrix>"
        "  </Physics>"
        "  <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='0' hi='180'/>"
        "    <PropagateJointValue factor='0.5' name='Joint2'/>"
        "    <MaxVelocity value='36' unitsLength='mm' unitsTime='h'/>"
        "    <MaxAcceleration value='36' unitsTime='min'/>"
        "    <MaxTorque value='0.2' units='meter'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "  <Transform>"
        "    <Translation x='100' y='50' z='0'/>"
        "  </Transform>"
        "   <Joint type='revolute'>"
        "    <axis x='0' y='0' z='1'/>"
        "    <Limits unit='degree' lo='0' hi='90'/>"
        "   </Joint>"
        "  <Sensor type='position' name='sensor2'>"
        "    <Transform>"
        "       <Translation x='100' y='50' z='0'/>"
        "    </Transfrom>"
        "  </Sensor>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node1 = "Joint1";
    const std::string node2 = "Joint2";
    VirtualRobot::RobotNodePtr rn1 = rob->getRobotNode(node1);
    BOOST_REQUIRE(rn1);
    VirtualRobot::RobotNodePtr rn2 = rob->getRobotNode(node2);
    BOOST_REQUIRE(rn2);

    // check physics
    float mass = rn1->getMass();
    BOOST_CHECK_EQUAL(mass, 100.0f);
    float vel = rn1->getMaxVelocity();
    BOOST_CHECK_CLOSE(vel, 1e-5f, 0.01f);
    float acc = rn1->getMaxAcceleration();
    BOOST_CHECK_CLOSE(acc, 0.01f, 0.01f);
    float to = rn1->getMaxTorque();
    BOOST_CHECK_CLOSE(to, 0.2f, 0.01f);
    Eigen::Vector3f com = rn1->getCoMLocal();
    bool comOK = com.isApprox(Eigen::Vector3f(10.0f, 20.0f, 30.0f));
    BOOST_REQUIRE(comOK);

    Eigen::Matrix3f inertia = rn1->getInertiaMatrix();
    Eigen::Matrix3f expectedMat;
    expectedMat << 0.001f, 0.002f, 0.003f, 0.004f, 0.005f, 0.006f, 0.007f, 0.008f, 0.009f;
    bool inertiaMatrixOK = inertia.isApprox(expectedMat);
    BOOST_REQUIRE(inertiaMatrixOK);


    // check sensor
    BOOST_REQUIRE(rn2->hasSensor("sensor2"));
    VirtualRobot::PositionSensorPtr ps = boost::dynamic_pointer_cast<VirtualRobot::PositionSensor>(rn2->getSensor("sensor2"));
    BOOST_REQUIRE(ps);
    Eigen::Matrix4f p = ps->getGlobalPose();
    Eigen::Matrix4f p2 = Eigen::Matrix4f::Identity();
    p2.block(0, 3, 3, 1) << 200.0f, 100.0f, 0;
    BOOST_REQUIRE(p.isApprox(p2));


    // create xml robot
    std::string robXML;
    BOOST_REQUIRE_NO_THROW(robXML = rob->toXML());
    BOOST_REQUIRE(!robXML.empty());

    VirtualRobot::RobotPtr rob2;
    BOOST_REQUIRE_NO_THROW(rob2 = VirtualRobot::RobotIO::createRobotFromString(robXML));
    BOOST_REQUIRE(rob2);

    rn1 = rob2->getRobotNode(node1);
    BOOST_REQUIRE(rn1);
    rn2 = rob2->getRobotNode(node2);
    BOOST_REQUIRE(rn2);

    // check physics
    mass = rn1->getMass();
    BOOST_CHECK_EQUAL(mass, 100.0f);
    vel = rn1->getMaxVelocity();
    BOOST_CHECK_CLOSE(vel, 1e-5f, 0.01f);
    acc = rn1->getMaxAcceleration();
    BOOST_CHECK_CLOSE(acc, 0.01f, 0.01f);
    to = rn1->getMaxTorque();
    BOOST_CHECK_CLOSE(to, 0.2f, 0.01f);
    com = rn1->getCoMLocal();
    comOK = com.isApprox(Eigen::Vector3f(10.0f, 20.0f, 30.0f));
    BOOST_REQUIRE(comOK);

    inertia = rn1->getInertiaMatrix();
    expectedMat << 0.001f, 0.002f, 0.003f, 0.004f, 0.005f, 0.006f, 0.007f, 0.008f, 0.009f;
    inertiaMatrixOK = inertia.isApprox(expectedMat);
    BOOST_REQUIRE(inertiaMatrixOK);


    // check sensor
    BOOST_REQUIRE(rn2->hasSensor("sensor2"));
    ps = boost::dynamic_pointer_cast<VirtualRobot::PositionSensor>(rn2->getSensor("sensor2"));
    BOOST_REQUIRE(ps);
    p = ps->getGlobalPose();
    p2 = Eigen::Matrix4f::Identity();
    p2.block(0, 3, 3, 1) << 200.0f, 100.0f, 0;
    BOOST_REQUIRE(p.isApprox(p2));
}

BOOST_AUTO_TEST_SUITE_END()
