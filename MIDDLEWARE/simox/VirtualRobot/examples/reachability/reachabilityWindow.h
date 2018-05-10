
#ifndef __reachabilityScene_WINDOW_H_
#define __reachabilityScene_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>
#include <VirtualRobot/Workspace/Reachability.h>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>


#include <vector>

#include "ui_reachabilityScene.h"

class reachabilityWindow : public QMainWindow
{
    Q_OBJECT
public:
    reachabilityWindow(std::string& sRobotFile, std::string& reachFile, Eigen::Vector3f& axisTCP, Qt::WFlags flags = 0);
    ~reachabilityWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);
    void resetSceneryAll();
    void selectRobot();
    void createReach();
    void saveReach();
    void loadReach();
    void fillHoles();
    void binarize();

    void collisionModel();
    void reachVisu();
    void selectRNS(int nr);
    void selectJoint(int nr);
    void jointValueChanged(int pos);
    void extendReach();

    //void showRobot();
    /*
    void showCoordSystem();
    void robotStructure();
    void robotCoordSystems();
    void robotFullModel();
    void closeHand();
    void openHand();
    void selectEEF(int nr);*/



    SoQtExaminerViewer* getExaminerViewer()
    {
        return m_pExViewer;
    };

protected:
    void loadRobot();

    void setupUI();
    QString formatString(const char* s, float f);
    void buildVisu();
    void updateRNSBox();
    void updateJointBox();
    void loadReachFile(std::string filename);
    /*
    void updateEEFBox();
    void displayTriangles();*/
    Ui::MainWindowReachability UI;
    SoQtExaminerViewer* m_pExViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotVisuSep;
    SoSeparator* reachabilityVisuSep;

    VirtualRobot::RobotPtr robot;
    std::string robotFile;
    std::string reachFile;
    Eigen::Vector3f axisTCP;
    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
    std::vector < VirtualRobot::RobotNodePtr > allRobotNodes;
    std::vector < VirtualRobot::RobotNodePtr > currentRobotNodes;
    std::vector < VirtualRobot::RobotNodeSetPtr > robotNodeSets;

    VirtualRobot::WorkspaceRepresentationPtr reachSpace;
    VirtualRobot::RobotNodePtr currentRobotNode;
    /*

    std::vector < VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::EndEffectorPtr currentEEF;
    ;*/


    bool useColModel;
    //bool structureEnabled;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualization;
};

#endif // __reachabilityScene_WINDOW_H_
