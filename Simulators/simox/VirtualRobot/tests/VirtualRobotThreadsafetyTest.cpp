/**
* @package    VirtualRobot
* @author     Stefan Ulbrich, Nikolaus Vahrenkamp
* @copyright  2010,2011,2012 Stefan Ulbrich, Nikolaus Vahrenkamp
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

VirtualRobot::RobotPtr rob;
VirtualRobot::RobotNodePtr r1;
VirtualRobot::RobotNodePtr r2;


void thread1()
{
    for (int i = 0; i < 20; i++)
    {
        float angle = float(i) / 100.0f * 90.0f - 45.0f;
        rob->setJointValue(r1, angle);
    }
}

void thread2()
{
    for (int i = 0; i < 20; i++)
    {
        r1->print();
        VirtualRobot::LinkedCoordinate coord(rob);
        coord.set(r1);
        std::cout << coord.getInFrame(r2).block<3, 1>(0, 3) << std::endl;
    }

}

BOOST_AUTO_TEST_CASE(testIntelligentCoordinate)
{
    const std::string robotString =
        "<Robot Type='MyDemoRobotType' StandardName='ExampleRobo' RootNode='Joint1'>"

        " <RobotNode name='Joint1'>"
        "    <Transform>"
        "     <Translation x='100' y='0' z='0'/>"
        "    </Transform>"
        "  <Joint type='revolute'>"
        "   <Limits unit='degree' lo='-45' hi='45'/>"
        "	<Axis x='1' y='0' z='0'/>"
        "  </Joint>"
        "  <Child name='Joint2'/>"
        " </RobotNode>"

        " <RobotNode name='Joint2'>"
        "    <Transform>"
        "     <Translation x='0' y='200' z='0'/>"
        "    </Transform>"
        "  <Joint type='revolute'>"
        "   <Limits unit='degree' lo='-45' hi='45'/>"
        "	<Axis x='0' y='0' z='1'/>"
        "  </Joint>"
        " </RobotNode>"

        "</Robot>";
    BOOST_REQUIRE_NO_THROW(rob = VirtualRobot::RobotIO::createRobotFromString(robotString));
    BOOST_REQUIRE(rob);

    const std::string node1 = "Joint1";
    const std::string node2 = "Joint2";
    r1 = rob->getRobotNode(node1);
    r2 = rob->getRobotNode(node2);
    BOOST_REQUIRE(r1);
    BOOST_REQUIRE(r2);

    rob->setThreadsafe(true);

    boost::thread t1(thread1);
    boost::thread t2(thread2);
    t1.join();
    t2.join();
    // design more advanced and complete tests

}

BOOST_AUTO_TEST_SUITE_END()
