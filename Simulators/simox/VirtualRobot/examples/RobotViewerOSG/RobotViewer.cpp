#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "showRobotWindow.h"

bool useColModel = false;


int main(int argc, char* argv[])
{
    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    cout << " --- START --- " << endl;
    std::string filename("robots/examples/RobotViewerOSG/Joint3DH.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filename = robFile;
        }
    }

    cout << "Using robot at " << filename << endl;

    QApplication app(argc, argv);

    showRobotWindow rw(filename);
    rw.show();
    return app.exec();
}
