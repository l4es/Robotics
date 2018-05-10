#include <VirtualRobot/Dynamics/dynamics.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>

using std::cout;
using namespace VirtualRobot;




int main(int argc, char* argv[])
{
    std::string filename("robots/ArmarIII/ArmarIII.xml");
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename);
    VirtualRobot::RuntimeEnvironment::processCommandLine(argc, argv);

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

    if (rob)
    {

        RobotNodeSetPtr ns = rob->getRobotNodeSet("RightArm");

        VirtualRobot::Dynamics dynamics = VirtualRobot::Dynamics(ns);
        int nDof = dynamics.getnDoF();
        Eigen::VectorXd q = Eigen::VectorXd::Random(nDof);
        Eigen::VectorXd qdot = Eigen::VectorXd::Random(nDof);
        Eigen::VectorXd qddot = Eigen::VectorXd::Random(nDof);
        Eigen::VectorXd tau = Eigen::VectorXd::Random(nDof);


        cout << "joint torques from inverse dynamics: " << endl << dynamics.getInverseDynamics(q, qdot, qddot) << endl;
        cout << "joint asdasdspace inertia matrix: " << endl << dynamics.getInertiaMatrix(q) << endl;
        cout << "joint space gravitational matrix:" << endl << dynamics.getGravityMatrix(q, nDof) << endl;
        cout << "joint space coriolis matrix:" << endl << dynamics.getCoriolisMatrix(q, qdot, nDof) << endl;
        cout << "joint space accelerations from forward dynamics:" << endl << dynamics.getForwardDynamics(q, qdot, tau) << endl;
        cout << "Identifier for Elbow R:" << endl << dynamics.getIdentifier("Elbow R") << endl;

    }
    else
    {
        cout << " ERROR while creating robobt" << endl;
    }
}



