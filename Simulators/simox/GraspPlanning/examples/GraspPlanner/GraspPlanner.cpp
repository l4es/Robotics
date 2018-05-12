#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "GraspPlannerWindow.h"


int main(int argc, char* argv[])
{
    SoDB::init();
    SoQt::init(argc, argv, "GraspPlanner");
    cout << " --- START --- " << endl;

    // --robot robots/iCub/iCub.xml --endeffector "Left Hand" --preshape "Grasp Preshape"
    std::string robot("robots/ArmarIII/ArmarIII.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robot);
    std::string eef("Hand R");
    //std::string object("objects/wok.xml");
    //std::string object("objects/riceBox.xml");
    std::string object("objects/WaterBottleSmall.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(object);
    std::string preshape("");

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("endeffector");
    VirtualRobot::RuntimeEnvironment::considerKey("preshape");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

    if (!robFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
    {
        robot = robFile;
    }

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");

    if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
    {
        object = objFile;
    }

    std::string eefname = VirtualRobot::RuntimeEnvironment::getValue("endeffector");

    if (!eefname.empty())
    {
        eef = eefname;
    }

    std::string ps = VirtualRobot::RuntimeEnvironment::getValue("preshape");

    if (!ps.empty())
    {
        preshape = ps;
    }


    cout << "Using robot from " << robot << endl;
    cout << "End effector:" << eef << ", preshape:" << preshape << endl;
    cout << "Using object from " << object << endl;

    GraspPlannerWindow rw(robot, eef, preshape, object);

    rw.main();

    return 0;
}
