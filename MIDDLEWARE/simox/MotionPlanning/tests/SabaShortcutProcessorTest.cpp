/**
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE Saba_SabaShortcutProcessorTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/CollisionDetection/CollisionChecker.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Obstacle.h>
#include <CSpace/CSpaceSampled.h>
#include <CSpace/CSpacePath.h>
#include <PostProcessing/ShortcutProcessor.h>
#include <VirtualRobot/CollisionDetection/CDManager.h>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>


BOOST_AUTO_TEST_SUITE(CSpaceShortcutProcessor)


BOOST_AUTO_TEST_CASE(testShortcutProcessor)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        "   <Joint type='revolute'>"
        "     <Limits unit='degree' lo='-180' hi='180'/>"
        "	  <Axis x='1' y='0' z='0'/>"
        "   </Joint>"
        "   <Child name='Joint2'/>"
        " </RobotNode>"
        " <RobotNode name='Joint2'>"
        "   <Joint type='revolute'>"
        "     <Limits unit='degree' lo='-180' hi='180'/>"
        "	  <Axis x='0' y='1' z='0'/>"
        "   </Joint>"
        " </RobotNode>"
        " <RobotNodeSet name='rns1'>"
        "  <Node name='Joint1'/>"
        "  <Node name='Joint2'/>"
        " </RobotNodeSet>"
        "</Robot>";
    VirtualRobot::RobotPtr rob = VirtualRobot::RobotIO::createRobotFromString(robotString);
    BOOST_REQUIRE(rob);
    VirtualRobot::RobotNodeSetPtr rns = rob->getRobotNodeSet("rns1");
    BOOST_REQUIRE(rns);
    VirtualRobot::CDManagerPtr cdm(new VirtualRobot::CDManager());
    Saba::CSpaceSampledPtr cspace(new Saba::CSpaceSampled(rob, cdm, rns));
    BOOST_REQUIRE(cspace);

    Eigen::VectorXf p1(2);
    Eigen::VectorXf p2(2);
    Eigen::VectorXf p3(2);
    Eigen::VectorXf p4(2);

    p1 << -1, 0;
    p2 << 0, 1;
    p3 << 1, 1;
    p4 << 1, 0;

    Saba::CSpacePathPtr path(new Saba::CSpacePath(cspace, "test_path"));
    BOOST_REQUIRE(path);

    path->addPoint(p1);
    path->addPoint(p2);
    path->addPoint(p3);
    path->addPoint(p4);

    Saba::ShortcutProcessorPtr sc(new Saba::ShortcutProcessor(path, cspace));
    BOOST_REQUIRE(sc);
    int loops = 100;
    int startI, endI;
    bool res;

    for (int i = 0; i < loops; i++)
    {
        res = sc->selectCandidatesRandom(startI, endI);
        BOOST_CHECK_EQUAL(res, true);
        bool startOK = startI == 0 || startI == 1;
        BOOST_CHECK_EQUAL(startOK, true);
        bool endOK = endI == 2 || endI == 3;
        BOOST_CHECK_EQUAL(endOK, true);
        bool distOK = (endI - startI) >= 2;
        BOOST_CHECK_EQUAL(distOK, true);
    }

    res = sc->validShortcut(0, 2);
    BOOST_CHECK_EQUAL(res, true);
    res = sc->validShortcut(0, 3);
    BOOST_CHECK_EQUAL(res, true);
    res = sc->validShortcut(1, 3);
    BOOST_CHECK_EQUAL(res, true);

    float l = path->getLength();
    Saba::CSpacePathPtr c1 = path->clone();
    Saba::CSpacePathPtr c2 = path->clone();
    Saba::CSpacePathPtr c3 = path->clone();
    Saba::ShortcutProcessorPtr sc1(new Saba::ShortcutProcessor(c1, cspace));
    Saba::ShortcutProcessorPtr sc2(new Saba::ShortcutProcessor(c2, cspace));
    Saba::ShortcutProcessorPtr sc3(new Saba::ShortcutProcessor(c3, cspace));

    sc1->doShortcut(0, 2);
    Saba::CSpacePathPtr o1 = sc1->getOptimizedPath();
    BOOST_REQUIRE(o1);
    float l1 = o1->getLength();
    BOOST_CHECK_LE(l1, l);

    sc2->doShortcut(0, 3);
    Saba::CSpacePathPtr o2 = sc2->getOptimizedPath();
    BOOST_REQUIRE(o2);
    float l2 = o2->getLength();
    BOOST_CHECK_LE(l2, l);

    sc3->doShortcut(1, 3);
    Saba::CSpacePathPtr o3 = sc3->getOptimizedPath();
    BOOST_REQUIRE(o3);
    float l3 = o3->getLength();
    BOOST_CHECK_LE(l3, l);


}

BOOST_AUTO_TEST_SUITE_END()
