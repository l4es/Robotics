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

#include "RrtGuiWindow.h"


int main(int argc, char* argv[])
{
    SoDB::init();
    SoQt::init(argc, argv, "RrtGui");
    cout << " --- START --- " << endl;

    std::string filenameScene("scenes/examples/RrtGui/planning.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameScene);
    std::string startConfig("start");
    std::string goalConfig("goal");
    std::string rnsName("Planning");
    std::string colModel1("ColModel Robot Moving");
    std::string colModel2("ColModel Robot Body");
    std::string colModel3("ColModel Obstacles");

    VirtualRobot::RuntimeEnvironment::considerKey("scene");
    VirtualRobot::RuntimeEnvironment::considerKey("startConfig");
    VirtualRobot::RuntimeEnvironment::considerKey("goalConfig");
    VirtualRobot::RuntimeEnvironment::considerKey("robotNodeSet");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelRobot1");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelRobot2");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelEnv");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string scFile = VirtualRobot::RuntimeEnvironment::getValue("scene");

    if (!scFile.empty() && VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(scFile))
    {
        filenameScene = scFile;
    }

    std::string sConf = VirtualRobot::RuntimeEnvironment::getValue("startConfig");

    if (!sConf.empty())
    {
        startConfig = sConf;
    }

    std::string gConf = VirtualRobot::RuntimeEnvironment::getValue("goalConfig");

    if (!gConf.empty())
    {
        goalConfig = gConf;
    }

    std::string rns = VirtualRobot::RuntimeEnvironment::getValue("robotNodeSet");

    if (!rns.empty())
    {
        rnsName = rns;
    }

    std::string c1 = VirtualRobot::RuntimeEnvironment::getValue("colModelRobot1");

    if (!c1.empty())
    {
        colModel1 = c1;
    }

    std::string c2 = VirtualRobot::RuntimeEnvironment::getValue("colModelRobot2");

    if (!c2.empty())
    {
        colModel2 = c2;
    }

    std::string c3 = VirtualRobot::RuntimeEnvironment::getValue("colModelEnv");

    if (!c3.empty())
    {
        colModel3 = c3;
    }


    cout << "Using scene: " << filenameScene << endl;
    cout << "Using start config: " << startConfig << endl;
    cout << "Using goal config: " << goalConfig << endl;
    cout << "Using RobotNodeSet for planning: " << rnsName << endl;
    cout << "Using robot collision model sets: " << colModel1 << " and " << colModel2 << endl;
    cout << "Using environment collision model set: " << colModel3 << endl;


    RrtGuiWindow rw(filenameScene, startConfig, goalConfig, rnsName, colModel1, colModel2, colModel3);

    rw.main();

    return 0;

}
