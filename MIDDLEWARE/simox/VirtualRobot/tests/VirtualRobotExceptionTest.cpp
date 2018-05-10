/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotExceptionTests

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_CASE(testVirtualRobotExceptionMacro)
{
    const std::string exceptionString = "Testing VR Exception";
    BOOST_CHECK_THROW(THROW_VR_EXCEPTION(exceptionString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotConditionalExceptionMacroThrow)
{
    const std::string exceptionString = "Testing VR Exception";
    BOOST_CHECK_THROW(THROW_VR_EXCEPTION_IF(true, exceptionString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testVirtualRobotConditionalExceptionMacroNoThrow)
{
    const std::string exceptionString = "Testing VR Exception";
    BOOST_CHECK_NO_THROW(THROW_VR_EXCEPTION_IF(false, exceptionString));
}
