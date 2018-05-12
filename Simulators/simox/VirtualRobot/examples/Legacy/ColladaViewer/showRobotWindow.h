
#ifndef __ShowRobot_WINDOW_H_
#define __ShowRobot_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>


#include <vector>

#include "ui_RobotViewer.h"

class showRobotWindow : public QMainWindow
{
    Q_OBJECT
public:
    showRobotWindow(std::string& sRobotFilename, Qt::WFlags flags = 0);
    ~showRobotWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();
    void rebuildVisualization();
    void showRobot();
    void loadRobot();
    void selectJoint(int nr);
    void selectRNS(int nr);
    void jointValueChanged(int pos);
    void showCoordSystem();
    void showSensors();
    void robotStructure();
    void robotCoordSystems();
    void robotFullModel();
    //void closeHand();
    //void openHand();
    //void selectEEF(int nr);
    void saveRobot();
    void selectRobot();
    void displayPhysics();


    SoQtExaminerViewer* getExaminerViewer()
    {
        return viewer;
    };

protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateJointBox();
    void updateRNSBox();
    //void updateEEFBox();
    void displayTriangles();
    Ui::MainWindowShowRobot UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;

    VirtualRobot::RobotPtr robot;
    std::string m_sRobotFilename;
    std::vector < VirtualRobot::RobotNodePtr > allRobotNodes;
    std::vector < VirtualRobot::RobotNodePtr > currentRobotNodes;
    std::vector < VirtualRobot::RobotNodeSetPtr > robotNodeSets;
    std::vector < VirtualRobot::EndEffectorPtr > eefs;
    VirtualRobot::EndEffectorPtr currentEEF;
    VirtualRobot::RobotNodeSetPtr currentRobotNodeSet;
    VirtualRobot::RobotNodePtr currentRobotNode;


    bool useColModel;
    bool structureEnabled;
    bool physicsCoMEnabled;
    bool physicsInertiaEnabled;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualization;
};

#endif // __ShowRobot_WINDOW_H_
