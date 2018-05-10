/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2013 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotIOTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/Sensor.h>
#include <VirtualRobot/Nodes/PositionSensor.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <string>


using namespace VirtualRobot;

BOOST_AUTO_TEST_SUITE(VirtualRobotIO)

BOOST_AUTO_TEST_CASE(testRobotLoadXML)
{
    std::string filename = "robots/ArmarIII/ArmarIII.xml";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    RobotPtr r;
    BOOST_REQUIRE_NO_THROW(r = RobotIO::loadRobot(filename));
    BOOST_REQUIRE(r);

    std::vector<RobotNodePtr> rn = r->getRobotNodes();
    BOOST_REQUIRE(rn.size() > 0);

    std::vector<EndEffectorPtr> eefs = r->getEndEffectors();
    BOOST_REQUIRE(eefs.size() > 0);

    std::vector<RobotNodeSetPtr> rns = r->getRobotNodeSets();
    BOOST_REQUIRE(rns.size() > 0);
}

BOOST_AUTO_TEST_CASE(testRobotSaveXML)
{
    std::string filename = "robots/ArmarIII/ArmarIII.xml";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    RobotPtr r;
    BOOST_REQUIRE_NO_THROW(r = RobotIO::loadRobot(filename));
    BOOST_REQUIRE(r);

    boost::system::error_code ec;
    boost::filesystem::path tempDir = boost::filesystem::temp_directory_path(ec);
    BOOST_REQUIRE(ec.value() == boost::system::errc::success);

    boost::filesystem::path robName("ArmarIII_tmp.xml");
    boost::filesystem::path filenameTmp = boost::filesystem::operator/(tempDir, robName);

    bool saveOK;
    BOOST_REQUIRE_NO_THROW(saveOK = RobotIO::saveXML(r, robName.string(), tempDir.string()));
    BOOST_REQUIRE(saveOK);

    //reload robot
    RobotPtr r2;
    BOOST_REQUIRE_NO_THROW(r2 = RobotIO::loadRobot(filenameTmp.string()));
    BOOST_REQUIRE(r2);

}


BOOST_AUTO_TEST_SUITE_END()
