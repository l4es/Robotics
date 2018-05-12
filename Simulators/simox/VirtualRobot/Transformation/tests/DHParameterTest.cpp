/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/

#define BOOST_TEST_MODULE VirtualRobot_DHParameterTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Transformation/DHParameter.h>

BOOST_AUTO_TEST_SUITE(Transformation)

BOOST_AUTO_TEST_CASE(DHParameterConstructorTest1)
{
    VirtualRobot::DHParameter dh;
    BOOST_CHECK_EQUAL(0, dh.thetaRadian());
    BOOST_CHECK_EQUAL(0, dh.dMM());
    BOOST_CHECK_EQUAL(0, dh.aMM());
    BOOST_CHECK_EQUAL(0, dh.alphaRadian());
    BOOST_CHECK_EQUAL(false, dh.isSet);
}

BOOST_AUTO_TEST_CASE(DHParameterConstructorTest2)
{
    VirtualRobot::DHParameter dh(90, 10, 30, 45, false);
    BOOST_CHECK_CLOSE(1.570, dh.thetaRadian(), 0.1);
    BOOST_CHECK_EQUAL(10, dh.dMM());
    BOOST_CHECK_EQUAL(30, dh.aMM());
    BOOST_CHECK_CLOSE(0.785, dh.alphaRadian(), 0.1);
    BOOST_CHECK_EQUAL(true, dh.isSet);
}

BOOST_AUTO_TEST_CASE(DHParameterSetThetaTest)
{
    VirtualRobot::DHParameter dh;
    BOOST_CHECK_EQUAL(0, dh.thetaRadian());
    dh.setThetaRadian(20, false);
    BOOST_CHECK_CLOSE(0.349, dh.thetaRadian(), 0.1);
}

BOOST_AUTO_TEST_CASE(DHParameterSetDTest)
{
    VirtualRobot::DHParameter dh;
    BOOST_CHECK_EQUAL(0, dh.dMM());
    dh.setDInMM(20);
    BOOST_CHECK_EQUAL(20, dh.dMM());
}

BOOST_AUTO_TEST_CASE(DHParameterSetATest)
{
    VirtualRobot::DHParameter dh;
    BOOST_CHECK_EQUAL(0, dh.aMM());
    dh.setAInMM(30);
    BOOST_CHECK_EQUAL(30, dh.aMM());
}

BOOST_AUTO_TEST_CASE(DHParameterSetAlphaTest)
{
    VirtualRobot::DHParameter dh;
    BOOST_CHECK_EQUAL(0, dh.alphaRadian());
    dh.setAlphaRadian(20, false);
    BOOST_CHECK_CLOSE(0.349, dh.alphaRadian(), 0.1);
}

BOOST_AUTO_TEST_CASE(DHParameterThetaRotationMatrixTest)
{
    VirtualRobot::DHParameter dh;
    float thetaDegree = 20.0f;
    float sinThetaRadian = 0.342f;
    float cosThetaRadian = 0.939f;
    // set new value
    dh.setThetaRadian(thetaDegree, false);
    // first row
    BOOST_CHECK_CLOSE(cosThetaRadian, dh.thetaRotationRadian()(0, 0), 0.1f);
    BOOST_CHECK_CLOSE(-sinThetaRadian, dh.thetaRotationRadian()(0, 1), 0.1f);
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(0, 2));
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(0, 3));
    // second row
    BOOST_CHECK_CLOSE(sinThetaRadian, dh.thetaRotationRadian()(1, 0), 0.1f);
    BOOST_CHECK_CLOSE(cosThetaRadian, dh.thetaRotationRadian()(1, 1), 0.1f);
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(1, 2));
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(1, 3));
    // third row
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(2, 0));
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(2, 1));
    BOOST_CHECK_EQUAL(1, dh.thetaRotationRadian()(2, 2));
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(2, 3));
    // fourth row
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(3, 0));
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(3, 1));
    BOOST_CHECK_EQUAL(0, dh.thetaRotationRadian()(3, 2));
    BOOST_CHECK_EQUAL(1, dh.thetaRotationRadian()(3, 3));
}

BOOST_AUTO_TEST_CASE(DHParameterDTranslationMatrixTest)
{
    VirtualRobot::DHParameter dh;
    float dInMM = 20;
    // set new value
    dh.setDInMM(dInMM);
    // first row
    BOOST_CHECK_EQUAL(1, dh.dTranslation()(0, 0));
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(0, 1));
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(0, 2));
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(0, 3));
    // second row
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(1, 0));
    BOOST_CHECK_EQUAL(1, dh.dTranslation()(1, 1));
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(1, 2));
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(1, 3));
    // third row
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(2, 0));
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(2, 1));
    BOOST_CHECK_EQUAL(1, dh.dTranslation()(2, 2));
    BOOST_CHECK_EQUAL(dInMM, dh.dTranslation()(2, 3));
    // fourth row
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(3, 0));
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(3, 1));
    BOOST_CHECK_EQUAL(0, dh.dTranslation()(3, 2));
    BOOST_CHECK_EQUAL(1, dh.dTranslation()(3, 3));
}

BOOST_AUTO_TEST_CASE(DHParameterATranslationMatrixTest)
{
    VirtualRobot::DHParameter dh;
    float aInMM = 20;
    // set new value
    dh.setAInMM(aInMM);
    // first row
    BOOST_CHECK_EQUAL(1, dh.aTranslation()(0, 0));
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(0, 1));
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(0, 2));
    BOOST_CHECK_EQUAL(aInMM, dh.aTranslation()(0, 3));
    // second row
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(1, 0));
    BOOST_CHECK_EQUAL(1, dh.aTranslation()(1, 1));
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(1, 2));
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(1, 3));
    // third row
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(2, 0));
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(2, 1));
    BOOST_CHECK_EQUAL(1, dh.aTranslation()(2, 2));
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(2, 3));
    // fourth row
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(3, 0));
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(3, 1));
    BOOST_CHECK_EQUAL(0, dh.aTranslation()(3, 2));
    BOOST_CHECK_EQUAL(1, dh.aTranslation()(3, 3));
}

BOOST_AUTO_TEST_CASE(DHParameterAlphaRotationMatrixTest)
{
    VirtualRobot::DHParameter dh;
    float alphaDegree = 20.0f;
    float sinAlphaRadian = 0.342f;
    float cosAlphaRadian = 0.939f;
    // set new value
    dh.setAlphaRadian(alphaDegree, false);
    // first row
    BOOST_CHECK_EQUAL(1, dh.alphaRotationRadian()(0, 0));
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(0, 1));
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(0, 2));
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(0, 3));
    // second row
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(1, 0));
    BOOST_CHECK_CLOSE(cosAlphaRadian, dh.alphaRotationRadian()(1, 1), 0.1f);
    BOOST_CHECK_CLOSE(-sinAlphaRadian, dh.alphaRotationRadian()(1, 2), 0.1f);
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(1, 3));
    // third row
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(2, 0));
    BOOST_CHECK_CLOSE(sinAlphaRadian, dh.alphaRotationRadian()(2, 1), 0.1f);
    BOOST_CHECK_CLOSE(cosAlphaRadian, dh.alphaRotationRadian()(2, 2), 0.1f);
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(2, 3));
    // fourth row
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(3, 0));
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(3, 1));
    BOOST_CHECK_EQUAL(0, dh.alphaRotationRadian()(3, 2));
    BOOST_CHECK_EQUAL(1, dh.alphaRotationRadian()(3, 3));
}

BOOST_AUTO_TEST_SUITE_END()
