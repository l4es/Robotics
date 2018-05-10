/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualTrajectoryTest

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Trajectory.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_SUITE(Trajectory)

BOOST_AUTO_TEST_CASE(testTrajectoryInvalidCreation)
{
    BOOST_REQUIRE_THROW(VirtualRobot::TrajectoryPtr c(new VirtualRobot::Trajectory(VirtualRobot::RobotNodeSetPtr(), "")), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testTrajectorySet)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        " </RobotNode>"
        " <RobotNodeSet name='rns1'>"
        "  <Node name='Joint1'/>"
        " </RobotNodeSet>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    VirtualRobot::RobotNodeSetPtr rns;
    BOOST_REQUIRE_NO_THROW(rns = rob->getRobotNodeSet("rns1"));
    BOOST_REQUIRE(rns);


    VirtualRobot::TrajectoryPtr t;
    BOOST_REQUIRE_NO_THROW(t.reset(new VirtualRobot::Trajectory(rns, "test")));
    BOOST_REQUIRE(t);

    Eigen::VectorXf a(1);
    a(0) = 0.0f;
    Eigen::VectorXf b(1);
    b(0) = 1.0f;
    BOOST_REQUIRE_NO_THROW(t->addPoint(a));
    BOOST_REQUIRE_NO_THROW(t->addPoint(b));
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), 2);
    Eigen::VectorXf a2 = t->getPoint(0);
    Eigen::VectorXf b2 = t->getPoint(1);
    BOOST_CHECK_EQUAL(a.isApprox(a2), true);
    BOOST_CHECK_EQUAL(b.isApprox(b2), true);

}


BOOST_AUTO_TEST_CASE(testTrajectoryClone)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        " </RobotNode>"
        " <RobotNodeSet name='rns1'>"
        "  <Node name='Joint1'/>"
        " </RobotNodeSet>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    VirtualRobot::RobotNodeSetPtr rns;
    BOOST_REQUIRE_NO_THROW(rns = rob->getRobotNodeSet("rns1"));
    BOOST_REQUIRE(rns);


    VirtualRobot::TrajectoryPtr t;
    BOOST_REQUIRE_NO_THROW(t.reset(new VirtualRobot::Trajectory(rns, "test")));
    BOOST_REQUIRE(t);

    // add some points
    int pts = 20;

    for (int i = 0; i < pts; i++)
    {
        Eigen::VectorXf a(1);
        a(0) = (float)i;
        BOOST_REQUIRE_NO_THROW(t->addPoint(a));
    }

    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts);

    VirtualRobot::TrajectoryPtr t1, t2, t3;
    BOOST_REQUIRE_NO_THROW(t1 = t->createSubPath(0, 1));
    BOOST_REQUIRE(t1);
    BOOST_CHECK_EQUAL(t1->getNrOfPoints(), 2);
    BOOST_CHECK_EQUAL(t1->getPoint(0)(0), 0.0f);
    BOOST_CHECK_EQUAL(t1->getPoint(1)(0), 1.0f);

    BOOST_REQUIRE_NO_THROW(t2 = t->createSubPath(5, 10));
    BOOST_REQUIRE(t2);
    BOOST_CHECK_EQUAL(t2->getNrOfPoints(), 6);
    BOOST_CHECK_EQUAL(t2->getPoint(0)(0), 5.0f);
    BOOST_CHECK_EQUAL(t2->getPoint(5)(0), 10.0f);

    BOOST_REQUIRE_NO_THROW(t3 = t->createSubPath(18, 19));
    BOOST_REQUIRE(t3);
    BOOST_CHECK_EQUAL(t3->getNrOfPoints(), 2);
    BOOST_CHECK_EQUAL(t3->getPoint(0)(0), 18.0f);
    BOOST_CHECK_EQUAL(t3->getPoint(1)(0), 19.0f);

    VirtualRobot::TrajectoryPtr tr;
    BOOST_REQUIRE_NO_THROW(tr = t->clone());
    BOOST_REQUIRE(tr);
    BOOST_CHECK_EQUAL(tr->getNrOfPoints(), pts);
    BOOST_REQUIRE_NO_THROW(tr->reverse());
    BOOST_CHECK_EQUAL(tr->getNrOfPoints(), pts);
    BOOST_CHECK_EQUAL(tr->getPoint(0)(0), (float)(pts - 1));
    BOOST_CHECK_EQUAL(tr->getPoint(pts - 1)(0), 0.0f);
}


BOOST_AUTO_TEST_CASE(testTrajectoryOperationsRemove)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        " </RobotNode>"
        " <RobotNodeSet name='rns1'>"
        "  <Node name='Joint1'/>"
        " </RobotNodeSet>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    VirtualRobot::RobotNodeSetPtr rns;
    BOOST_REQUIRE_NO_THROW(rns = rob->getRobotNodeSet("rns1"));
    BOOST_REQUIRE(rns);


    VirtualRobot::TrajectoryPtr t;
    BOOST_REQUIRE_NO_THROW(t.reset(new VirtualRobot::Trajectory(rns, "test")));
    BOOST_REQUIRE(t);

    // add some points
    int pts = 20;

    for (int i = 0; i < pts; i++)
    {
        Eigen::VectorXf a(1);
        a(0) = (float)i;
        BOOST_REQUIRE_NO_THROW(t->addPoint(a));
    }

    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 0.0f);

    BOOST_REQUIRE_NO_THROW(t->erasePosition(0)); // 1 2 3 ... 18 19
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts - 1);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 1.0f);

    BOOST_REQUIRE_NO_THROW(t->erasePosition(1)); // 1 3 4 ... 18 19
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts - 2);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 1.0f);
    BOOST_CHECK_EQUAL(t->getPoint(1)(0), 3.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts - 3)(0), (float)(pts - 1));

    BOOST_REQUIRE_NO_THROW(t->erasePosition(pts - 3)); // 1 3 4 ... 17 18
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts - 3);
    BOOST_CHECK_EQUAL(t->getPoint(pts - 4)(0), (float)(pts - 2));

    BOOST_REQUIRE_NO_THROW(t->erasePosition(pts - 5)); // 1 3 4 ... 16 18
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts - 4);
    BOOST_CHECK_EQUAL(t->getPoint(pts - 5)(0), (float)(pts - 2));
    BOOST_CHECK_EQUAL(t->getPoint(pts - 6)(0), (float)(pts - 4));

    BOOST_REQUIRE_NO_THROW(t->reset());
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), 0);

    for (int i = 0; i < pts; i++)
    {
        Eigen::VectorXf a(1);
        a(0) = (float)i;
        BOOST_REQUIRE_NO_THROW(t->addPoint(a));
    }

    // 0 1 2 3 4 ... 18 19
    BOOST_REQUIRE_NO_THROW(t->removePositions(0, 2)); // 3 4 5... 18 19
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts - 3);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 3.0f);

    BOOST_REQUIRE_NO_THROW(t->removePositions(pts - 5, pts - 4)); // 3 4 5 ... 16 17
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts - 5);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 3.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts - 6)(0), 17.0f);

    BOOST_REQUIRE_NO_THROW(t->removePositions(2, 4)); // 3 4 8 ... 16 17
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts - 8);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 3.0f);
    BOOST_CHECK_EQUAL(t->getPoint(1)(0), 4.0f);
    BOOST_CHECK_EQUAL(t->getPoint(2)(0), 8.0f);
}


BOOST_AUTO_TEST_CASE(testTrajectoryOperationsInsert)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' RootNode='Joint1'>"
        " <RobotNode name='Joint1'>"
        " </RobotNode>"
        " <RobotNodeSet name='rns1'>"
        "  <Node name='Joint1'/>"
        " </RobotNodeSet>"
        "</Robot>";
    VirtualRobot::RobotPtr rob;
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    VirtualRobot::RobotNodeSetPtr rns;
    BOOST_REQUIRE_NO_THROW(rns = rob->getRobotNodeSet("rns1"));
    BOOST_REQUIRE(rns);


    VirtualRobot::TrajectoryPtr t;
    BOOST_REQUIRE_NO_THROW(t.reset(new VirtualRobot::Trajectory(rns, "test")));
    BOOST_REQUIRE(t);

    // add some points
    int pts = 20;

    for (int i = 0; i < pts; i++)
    {
        Eigen::VectorXf a(1);
        a(0) = (float)i;
        BOOST_REQUIRE_NO_THROW(t->addPoint(a));
    }

    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 0.0f);

    Eigen::VectorXf b(1);
    b(0) = 100.0f;
    BOOST_REQUIRE_NO_THROW(t->insertPosition(0, b)); // 100 0 1 2 3 ... 18 19
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts + 1);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 100.0f);
    BOOST_CHECK_EQUAL(t->getPoint(1)(0), 0.0f);

    Eigen::VectorXf c(1);
    c(0) = 200.0f;
    BOOST_REQUIRE_NO_THROW(t->insertPosition(pts + 1, c)); // 100 0 1 2 3 ... 18 19 200
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts + 2);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 100.0f);
    BOOST_CHECK_EQUAL(t->getPoint(1)(0), 0.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts)(0), 19.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 1)(0), 200.0f);

    Eigen::VectorXf d(1);
    d(0) = 300.0f;
    BOOST_REQUIRE_NO_THROW(t->insertPosition(3, d)); // 100 0 1 300 2 3 ... 18 19 200
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts + 3);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 100.0f);
    BOOST_CHECK_EQUAL(t->getPoint(1)(0), 0.0f);
    BOOST_CHECK_EQUAL(t->getPoint(2)(0), 1.0f);
    BOOST_CHECK_EQUAL(t->getPoint(3)(0), 300.0f);
    BOOST_CHECK_EQUAL(t->getPoint(4)(0), 2.0f);


    VirtualRobot::TrajectoryPtr t2;
    BOOST_REQUIRE_NO_THROW(t2.reset(new VirtualRobot::Trajectory(rns, "test")));
    BOOST_REQUIRE(t2);

    // add some points
    int pts2 = 3;

    for (int i = 0; i < pts2; i++)
    {
        Eigen::VectorXf a(1);
        a(0) = (float)i * 1000.0f;
        BOOST_REQUIRE_NO_THROW(t2->addPoint(a));
    }

    BOOST_CHECK_EQUAL(t2->getNrOfPoints(), pts2);

    BOOST_REQUIRE_NO_THROW(t->insertTrajectory(0, t2)); // 0 1000 2000 100 0 1 300 2 3 ... 18 19 200
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts + 6);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 0.0f);
    BOOST_CHECK_EQUAL(t->getPoint(1)(0), 1000.0f);
    BOOST_CHECK_EQUAL(t->getPoint(2)(0), 2000.0f);
    BOOST_CHECK_EQUAL(t->getPoint(3)(0), 100.0f);

    BOOST_REQUIRE_NO_THROW(t->insertTrajectory(pts + 6, t2)); // 0 1000 2000 100 0 1 300 2 3 ... 18 19 200 0 1000 2000
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts + 9);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 8)(0), 2000.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 7)(0), 1000.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 6)(0), 0.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 5)(0), 200.0f);

    std::vector< Eigen::VectorXf > pt;
    Eigen::VectorXf e(1);
    e(0) = 111.0f;
    pt.push_back(e);
    e(0) = 222.0f;
    pt.push_back(e);
    e(0) = 333.0f;
    pt.push_back(e);

    BOOST_REQUIRE_NO_THROW(t->insertPosition(0, pt)); // 111 222 333 0 1000 2000 100 0 1 300 2 3 ... 18 19 200
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts + 12);
    BOOST_CHECK_EQUAL(t->getPoint(0)(0), 111.0f);
    BOOST_CHECK_EQUAL(t->getPoint(1)(0), 222.0f);
    BOOST_CHECK_EQUAL(t->getPoint(2)(0), 333.0f);
    BOOST_CHECK_EQUAL(t->getPoint(3)(0), 0.0f);

    BOOST_REQUIRE_NO_THROW(t->insertPosition(pts + 12, pt)); //  111 222 333 0 1000 2000 100 0 1 300 2 3 ... 18 19 200 0 1000 2000 111 222 333
    BOOST_CHECK_EQUAL(t->getNrOfPoints(), pts + 15);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 14)(0), 333.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 13)(0), 222.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 12)(0), 111.0f);
    BOOST_CHECK_EQUAL(t->getPoint(pts + 11)(0), 2000.0f);

}




BOOST_AUTO_TEST_SUITE_END()
