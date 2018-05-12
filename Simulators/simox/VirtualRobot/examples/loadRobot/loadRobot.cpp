#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Nodes/RobotNodeRevoluteFactory.h>
#include <VirtualRobot/Transformation/DHParameter.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>


#include <string>
#include <iostream>

using std::cout;
using std::endl;
using namespace VirtualRobot;


int main(int argc, char* argv[])
{
    boost::shared_ptr<Robot> robot = RobotFactory::createRobot("Robbi");
    std::vector< boost::shared_ptr<RobotNode> > robotNodes;
    VirtualRobot::RobotNodeRevoluteFactory revoluteNodeFactory;
    DHParameter dhParameter(0, 0, 0, 0, true);
    boost::shared_ptr<RobotNode> node1 = revoluteNodeFactory.createRobotNodeDH(robot, "RootNode", VisualizationNodePtr(), CollisionModelPtr(), (float) - M_PI, (float)M_PI, 0.0f, dhParameter);
    robotNodes.push_back(node1);
    std::map<RobotNodePtr, std::vector<std::string> > childrenMap;
    bool resInit = RobotFactory::initializeRobot(robot, robotNodes, childrenMap, node1);

    cout << "resInit:" << resInit << endl;
    cout << "First robot:" << endl;
    robot->print();

    VirtualRobot::RuntimeEnvironment::considerKey("robot");
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);
    VirtualRobot::RuntimeEnvironment::print();

    std::string filename("robots/examples/loadRobot/RobotExample.xml");
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
    RobotPtr rob;

    try
    {
        rob = RobotIO::loadRobot(filename, RobotIO::eStructure);
    }
    catch (VirtualRobotException& e)
    {
        cout << "Error: " << e.what() << endl;
        return -1;
    }

    cout << "Second robot (XML):" << endl;

    if (rob)
    {
        rob->print();
    }
    else
    {
        cout << " ERROR while creating robot" << endl;
    }
}
