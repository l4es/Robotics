#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/RuntimeEnvironment.h>



#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/Qt/SoQt.h>

#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "showCamWindow.h"

bool useColModel = false;


int main(int argc, char* argv[])
{
    SoDB::init();
    SoQt::init(argc, argv, "RobotViewer");


    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::considerKey("cam1");
    VirtualRobot::RuntimeEnvironment::considerKey("cam2");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    cout << " --- START --- " << endl;

    std::string filename("robots/ArmarIII/ArmarIII.xml");
    std::string cam1Name("EyeLeftCamera");
    std::string cam2Name("EyeRightCamera");

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        std::string robFile = VirtualRobot::RuntimeEnvironment::getValue("robot");

        if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFile))
        {
            filename = robFile;
        }
    }

    if (VirtualRobot::RuntimeEnvironment::hasValue("cam1"))
        cam1Name = VirtualRobot::RuntimeEnvironment::getValue("cam1");
    if (VirtualRobot::RuntimeEnvironment::hasValue("cam2"))
        cam2Name = VirtualRobot::RuntimeEnvironment::getValue("cam2");


    cout << "Using robot:" << filename << ", cam1:" << cam1Name << ", cam2:" << cam2Name << endl;

    showCamWindow rw(filename,cam1Name,cam2Name);

    rw.main();

    return 0;

}
