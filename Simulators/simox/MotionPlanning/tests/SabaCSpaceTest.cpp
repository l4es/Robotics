/**
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE Saba_SabaCSpaceTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Obstacle.h>
#include <CSpace/CSpaceSampled.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>


BOOST_AUTO_TEST_SUITE(CSpace)


BOOST_AUTO_TEST_CASE(testCSpace)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "   <Joint type='revolute'>"
        "    <Limits unit='degree' lo='-180' hi='180'/>"
        "	  <Axis x='1' y='0' z='0'/>"
        "   </Joint>"
        " </RobotNode>"
        "</Robot>";
    VirtualRobot::RobotPtr rob = VirtualRobot::RobotIO::createRobotFromString(robotString);
    BOOST_REQUIRE(rob);
    std::vector< std::string > nodes;
    nodes.push_back(std::string("Joint1"));
    VirtualRobot::RobotNodeSetPtr rns = VirtualRobot::RobotNodeSet::createRobotNodeSet(rob, "nodeSet", nodes);
    VirtualRobot::CDManagerPtr cdm(new VirtualRobot::CDManager());
    Saba::CSpaceSampledPtr cspace(new Saba::CSpaceSampled(rob, cdm, rns));
    BOOST_REQUIRE(cspace);

    Eigen::VectorXf p1(1);
    Eigen::VectorXf p2(1);
    float d;

    // ---distance tests---
    p1(0) = 0;
    p2(0) = 0.5f;
    d = cspace->calcDist(p1, p2);
    BOOST_CHECK_CLOSE(d, 0.5f, 0.01f);

    p1(0) = 0;
    p2(0) = -0.5f;
    d = cspace->calcDist(p1, p2);
    BOOST_CHECK_CLOSE(d, 0.5f, 0.01f);

    p1(0) = 0.5;
    p2(0) = -0.5f;
    d = cspace->calcDist(p1, p2);
    BOOST_CHECK_CLOSE(d, 1.0f, 0.01f);

    p1(0) = -0.5;
    p2(0) = 0.5f;
    d = cspace->calcDist(p1, p2);
    BOOST_CHECK_CLOSE(d, 1.0f, 0.01f);

    // traverse border
    p1(0) = -0.75f * (float)M_PI;
    p2(0) = 0.75f * (float)M_PI;
    d = cspace->calcDist(p1, p2);
    BOOST_CHECK_CLOSE(d, (float)M_PI * 0.5f, 0.01f);

    p1(0) = 0.75f * (float)M_PI;
    p2(0) = -0.75f * (float)M_PI;
    d = cspace->calcDist(p1, p2);
    BOOST_CHECK_CLOSE(d, (float)M_PI * 0.5f, 0.01f);

    // ---interpolation tests---
    p1(0) = 0;
    p2(0) = 0.5f;
    d = cspace->interpolate(p1, p2, 0, 0);
    BOOST_CHECK_CLOSE(d, 0.0f, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 1);
    BOOST_CHECK_CLOSE(d, 0.5f, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 0.5f);
    BOOST_CHECK_CLOSE(d, 0.25f, 0.01f);

    p1(0) = 0;
    p2(0) = -0.5f;
    d = cspace->interpolate(p1, p2, 0, 0);
    BOOST_CHECK_CLOSE(d, 0.0f, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 1);
    BOOST_CHECK_CLOSE(d, -0.5f, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 0.5f);
    BOOST_CHECK_CLOSE(d, -0.25f, 0.01f);

    p1(0) = 0.5;
    p2(0) = -0.5f;
    d = cspace->interpolate(p1, p2, 0, 0);
    BOOST_CHECK_CLOSE(d, 0.5f, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 1);
    BOOST_CHECK_CLOSE(d, -0.5f, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 0.5f);
    BOOST_CHECK_CLOSE(d, 0.0f, 0.01f);

    p1(0) = -0.5;
    p2(0) = 0.5f;
    d = cspace->interpolate(p1, p2, 0, 0);
    BOOST_CHECK_CLOSE(d, -0.5f, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 1);
    BOOST_CHECK_CLOSE(d, 0.5f, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 0.5f);
    BOOST_CHECK_CLOSE(d, 0.0f, 0.01f);

    // traverse border
    p1(0) = -0.75f * (float)M_PI;
    p2(0) = 0.75f * (float)M_PI;
    d = cspace->interpolate(p1, p2, 0, 0);
    BOOST_CHECK_CLOSE(d, -0.75f * (float)M_PI, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 1);
    BOOST_CHECK_CLOSE(d, 0.75f * (float)M_PI, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 0.1f); // dist is 0.5PI -> result (-0.75 - 0.1*0.5) * PI
    BOOST_CHECK_CLOSE(d, -0.75f * (float)M_PI - 0.1f * 0.5f * (float)M_PI, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 0.4f);
    BOOST_CHECK_CLOSE(d, -0.75f * (float)M_PI - 0.4f * 0.5f * (float)M_PI, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 0.6f);
    BOOST_CHECK_CLOSE(d, 0.75f * (float)M_PI + 0.4f * 0.5f * (float)M_PI, 0.01f);
    d = cspace->interpolate(p1, p2, 0, 0.9f);
    BOOST_CHECK_CLOSE(d, 0.75f * (float)M_PI + 0.1f * 0.5f * (float)M_PI, 0.01f);

}

BOOST_AUTO_TEST_SUITE_END()
