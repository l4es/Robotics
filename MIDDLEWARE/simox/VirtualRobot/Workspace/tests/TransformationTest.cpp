/**
* @package    VirtualRobot
* @author     Manfred Kroehnert <mkroehnert _at_ users dot sourceforge dot net>
* @copyright  2010 Manfred Kroehnert
*/

#define BOOST_TEST_MODULE VirtualRobot_TransformationTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Transformation/Transformation.h>
#include <Eigen/Core>

BOOST_AUTO_TEST_SUITE(Transformation)

void checkMatrix4fIdentity(const Eigen::Matrix4f& matrix)
{
    // first row
    BOOST_CHECK_EQUAL(1, matrix(0, 0));
    BOOST_CHECK_EQUAL(0, matrix(0, 1));
    BOOST_CHECK_EQUAL(0, matrix(0, 2));
    BOOST_CHECK_EQUAL(0, matrix(0, 3));
    // second row
    BOOST_CHECK_EQUAL(0, matrix(1, 0));
    BOOST_CHECK_EQUAL(1, matrix(1, 1));
    BOOST_CHECK_EQUAL(0, matrix(1, 2));
    BOOST_CHECK_EQUAL(0, matrix(1, 3));
    // third row
    BOOST_CHECK_EQUAL(0, matrix(2, 0));
    BOOST_CHECK_EQUAL(0, matrix(2, 1));
    BOOST_CHECK_EQUAL(1, matrix(2, 2));
    BOOST_CHECK_EQUAL(0, matrix(2, 3));
    // fourth row
    BOOST_CHECK_EQUAL(0, matrix(3, 0));
    BOOST_CHECK_EQUAL(0, matrix(3, 1));
    BOOST_CHECK_EQUAL(0, matrix(3, 2));
    BOOST_CHECK_EQUAL(1, matrix(3, 3));
}

void checkVector3fIdentity(const Eigen::Vector3f& vector)
{
    BOOST_CHECK_EQUAL(1, vector.x());
    BOOST_CHECK_EQUAL(0, vector.y());
    BOOST_CHECK_EQUAL(0, vector.z());
}

BOOST_AUTO_TEST_CASE(TransformationConstructorTest)
{
    VirtualRobot::Transformation transform;
    checkVector3fIdentity(transform.getJointRotationAxis());
    checkVector3fIdentity(transform.getJointTranslationDirection());
    checkMatrix4fIdentity(transform.getPreJointTransformation());
    checkMatrix4fIdentity(transform.getPostJointTransformation());
}

BOOST_AUTO_TEST_SUITE_END()
