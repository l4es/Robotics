/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotSceneTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Scene.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <string>

BOOST_AUTO_TEST_SUITE(Scene)

BOOST_AUTO_TEST_CASE(testSceneEmptyXML)
{
    const std::string sceneString = "";
    BOOST_REQUIRE_THROW((VirtualRobot::SceneIO::createSceneFromString(sceneString)), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testSceneUnclosedSceneTag)
{
    const std::string sceneString = "<Scene name='test'>";
    VirtualRobot::ScenePtr scene;
    BOOST_REQUIRE_THROW(scene = VirtualRobot::SceneIO::createSceneFromString(sceneString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testSceneOnlyClosedSceneTag)
{
    const std::string sceneString = "</Scene name='test'>";
    BOOST_REQUIRE_THROW(VirtualRobot::SceneIO::createSceneFromString(sceneString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testSceneEmptySceneTag)
{
    const std::string sceneString = "<Scene></Scene>";
    BOOST_REQUIRE_THROW(VirtualRobot::SceneIO::createSceneFromString(sceneString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testSceneEmptyNameString)
{
    const std::string sceneString = "<Scene name=''></Scene>";
    BOOST_REQUIRE_THROW(VirtualRobot::SceneIO::createSceneFromString(sceneString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testSceneInvalidRobot1Type)
{
    const std::string sceneString = "<Scene name='test'><Robot></Robot></Scene>";
    BOOST_REQUIRE_THROW(VirtualRobot::SceneIO::createSceneFromString(sceneString), VirtualRobot::VirtualRobotException);
}
BOOST_AUTO_TEST_CASE(testSceneInvalidRobot2Type)
{
    const std::string sceneString = "<Scene name='test'><Robot name='testRob'></Robot></Scene>";
    BOOST_REQUIRE_THROW(VirtualRobot::SceneIO::createSceneFromString(sceneString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testSceneInvalidRobot3Type)
{
    const std::string sceneString = "<Scene name='test'><Robot name='testRob'><File></File></Robot></Scene>";
    BOOST_REQUIRE_THROW(VirtualRobot::SceneIO::createSceneFromString(sceneString), VirtualRobot::VirtualRobotException);
}

BOOST_AUTO_TEST_CASE(testSceneInvalidRobot4Type)
{
    const std::string sceneString = "<Scene name='test'><Robot name='testRob'><File>thisisnotarobot.xml</File></Robot></Scene>";
    BOOST_REQUIRE_THROW(VirtualRobot::SceneIO::createSceneFromString(sceneString), VirtualRobot::VirtualRobotException);
}


BOOST_AUTO_TEST_SUITE_END()
