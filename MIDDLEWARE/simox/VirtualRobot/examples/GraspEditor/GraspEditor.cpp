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

#include <VirtualRobot/Grasping/GraspEditor/GraspEditorWindow.h>


int main(int argc, char* argv[])
{
    SoDB::init();
    SoQt::init(argc, argv, "GraspEditor");
    cout << " --- START --- " << endl;

    std::string filename1("objects/plate.xml");
    std::string filename2("robots/ArmarIII/ArmarIII.xml");
#if 0
    filename1 = "objects/iCub/LegoXWing_RightHand_300.xml";
    filename2 = "robots/iCub/iCub.xml";
#endif
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename1);
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename2);

    VirtualRobot::RuntimeEnvironment::considerKey("object");
    VirtualRobot::RuntimeEnvironment::considerKey("robot");

    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string objFile = VirtualRobot::RuntimeEnvironment::getValue("object");

    if (!objFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(objFile))
    {
        filename1 = objFile;
    }

    if (VirtualRobot::RuntimeEnvironment::hasValue("robot"))
    {
        filename2 = VirtualRobot::RuntimeEnvironment::getValue("robot");
    }

    GraspEditorWindow rw(filename1, filename2);

    rw.main();

    return 0;

}
