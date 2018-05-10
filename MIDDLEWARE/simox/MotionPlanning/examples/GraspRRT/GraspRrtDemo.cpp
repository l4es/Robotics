
#include "GraspRrtWindow.h"

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


int main(int argc, char** argv)
{
    SoDB::init();
    SoQt::init(argc, argv, "GraspRrtDemo");
    cout << " --- START --- " << endl;

    std::string filenameScene("/scenes/examples/GraspRrt/planning.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filenameScene);
    std::string startConfig("init");
    std::string goalObject("Can");
    std::string rnsName("Planning Left");
    std::string colModel1("ColModel Robot Moving Left");
    std::string colModel2("ColModel Robot Body1");
    std::string colModel3("ColModel Obstacles");
    std::string eefName("Hand L");

    std::string rnsNameB("Planning Right");
    std::string colModel1B("ColModel Robot Moving Right");
    std::string colModel2B("ColModel Robot Body2");
    std::string eefNameB("Hand R");

    VirtualRobot::RuntimeEnvironment::considerKey("scene");
    VirtualRobot::RuntimeEnvironment::considerKey("startConfig");
    VirtualRobot::RuntimeEnvironment::considerKey("targetObject");
    VirtualRobot::RuntimeEnvironment::considerKey("robotNodeSet_A");
    VirtualRobot::RuntimeEnvironment::considerKey("eef_A");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelRobot1_A");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelRobot2_A");
    VirtualRobot::RuntimeEnvironment::considerKey("robotNodeSet_B");
    VirtualRobot::RuntimeEnvironment::considerKey("eef_B");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelRobot1_B");
    VirtualRobot::RuntimeEnvironment::considerKey("colModelRobot2_B");
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

    std::string gConf = VirtualRobot::RuntimeEnvironment::getValue("targetObject");

    if (!gConf.empty())
    {
        goalObject = gConf;
    }

    std::string rns = VirtualRobot::RuntimeEnvironment::getValue("robotNodeSet_A");

    if (!rns.empty())
    {
        rnsName = rns;
    }

    std::string eef = VirtualRobot::RuntimeEnvironment::getValue("eef_A");

    if (!eef.empty())
    {
        eefName = eef;
    }

    std::string c1 = VirtualRobot::RuntimeEnvironment::getValue("colModelRobot1_A");

    if (!c1.empty())
    {
        colModel1 = c1;
    }

    std::string c2 = VirtualRobot::RuntimeEnvironment::getValue("colModelRobot2_A");

    if (!c2.empty())
    {
        colModel2 = c2;
    }

    rns = VirtualRobot::RuntimeEnvironment::getValue("robotNodeSet_B");

    if (!rns.empty())
    {
        rnsNameB = rns;
    }

    eef = VirtualRobot::RuntimeEnvironment::getValue("eef_B");

    if (!eef.empty())
    {
        eefNameB = eef;
    }

    c1 = VirtualRobot::RuntimeEnvironment::getValue("colModelRobot1_B");

    if (!c1.empty())
    {
        colModel1B = c1;
    }

    c2 = VirtualRobot::RuntimeEnvironment::getValue("colModelRobot2_B");

    if (!c2.empty())
    {
        colModel2B = c2;
    }

    std::string c3 = VirtualRobot::RuntimeEnvironment::getValue("colModelEnv");

    if (!c3.empty())
    {
        colModel3 = c3;
    }


    cout << "Using scene: " << filenameScene << endl;
    cout << "Using start config: <" << startConfig << ">" << endl;
    cout << "Using target object: <" << goalObject << ">" << endl;
    cout << "Using environment collision model set: <" << colModel3 << ">" << endl;
    cout << "Set 1:" << endl;
    cout << "\t Using RobotNodeSet for planning: <" << rnsName << ">" << endl;
    cout << "\t Using EEF for grasping: <" << eefName << ">" << endl;
    cout << "\t Using robot collision model sets: <" << colModel1 << "> and <" << colModel2 << ">" << endl;
    cout << "Set 2:" << endl;
    cout << "\t Using RobotNodeSet for planning: <" << rnsNameB << ">" << endl;
    cout << "\t Using EEF for grasping: <" << eefNameB << ">" << endl;
    cout << "\t Using robot collision model sets: <" << colModel1B << "> and <" << colModel2B << ">" << endl;


    GraspRrtWindow rw(filenameScene, startConfig, goalObject, rnsName, rnsNameB, eefName, eefNameB, colModel1, colModel1B, colModel2, colModel2B, colModel3);

    rw.main();

    return 0;
}
