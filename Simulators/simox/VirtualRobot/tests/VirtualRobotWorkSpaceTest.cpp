/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotWorkSpaceTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Workspace/WorkspaceRepresentation.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/MathTools.h>
#include <string>

BOOST_AUTO_TEST_SUITE(WorkSpace)

BOOST_AUTO_TEST_CASE(testWorksSpaceEuler)
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
    VirtualRobot::WorkspaceRepresentationPtr ws;
    BOOST_REQUIRE_NO_THROW(ws.reset(new VirtualRobot::WorkspaceRepresentation(rob)));
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    float x[6];

    // identity, matrix -> vector
    ws->matrix2Vector(m, x);

    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[3], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[4], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[5], 0.0f, 1e-6f);

    // identity, vector -> matrix
    for (int i = 0; i < 6; i++)
    {
        x[i] = 0.0f;
    }

    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    Eigen::Vector3f ax;
    float a;
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, 0.0f, 1e-6f);

    // rot x
    m.setIdentity();
    Eigen::Matrix3f m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, Eigen::Vector3f::UnitX()).matrix();
    m.block(0, 0, 3, 3) = m3;


    ws->matrix2Vector(m, x);
    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, float(M_PI) / 4.0f, 1e-3f);
    BOOST_CHECK_CLOSE(ax(0), 1.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(1), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(2), 0.0f, 1e-6f);


    // rot y
    m.setIdentity();
    m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, Eigen::Vector3f::UnitY()).matrix();
    m.block(0, 0, 3, 3) = m3;
    ws->matrix2Vector(m, x);
    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, float(M_PI) / 4.0f, 1e-3f);
    BOOST_CHECK_CLOSE(ax(1), 1.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(0), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(2), 0.0f, 1e-6f);

    // rot z
    m.setIdentity();
    m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, Eigen::Vector3f::UnitZ()).matrix();
    m.block(0, 0, 3, 3) = m3;
    ws->matrix2Vector(m, x);
    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, float(M_PI) / 4.0f, 1e-3f);
    BOOST_CHECK_CLOSE(ax(2), 1.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(1), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(ax(0), 0.0f, 1e-6f);


    // rot x,y
    m.setIdentity();
    ax << 1.0f, 1.0f, 0.0f;
    ax.normalize();
    m3 = Eigen::AngleAxisf(float(M_PI) / 4.0f, ax).matrix();
    m.block(0, 0, 3, 3) = m3;
    ws->matrix2Vector(m, x);
    ws->vector2Matrix(x, m);
    BOOST_CHECK_CLOSE(x[0], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[1], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(x[2], 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(0, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(1, 3), 0.0f, 1e-6f);
    BOOST_CHECK_CLOSE(m(2, 3), 0.0f, 1e-6f);
    VirtualRobot::MathTools::eigen4f2axisangle(m, ax, a);
    BOOST_CHECK_CLOSE(a, float(M_PI) / 4.0f, 1e-3f);
    BOOST_CHECK_CLOSE(ax(0), 1.0f / sqrt(2.0f), 1e-3f);
    BOOST_CHECK_CLOSE(ax(1), 1.0f / sqrt(2.0f), 1e-3f);
    BOOST_CHECK_SMALL(ax(2), 1e-4f);

}

BOOST_AUTO_TEST_SUITE_END()
