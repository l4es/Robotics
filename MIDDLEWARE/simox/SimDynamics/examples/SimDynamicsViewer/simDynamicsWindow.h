
#ifndef __SimDynamics_WINDOW_H_
#define __SimDynamics_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Obstacle.h>
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h"
#include <VirtualRobot/Nodes/RobotNodeRevolute.h>

#include "SimDynamics/DynamicsEngine/BulletEngine/BulletCoinQtViewer.h"

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

#include "ui_simDynamicsViewer.h"

class SimDynamicsWindow : public QMainWindow
{
    Q_OBJECT
public:
    SimDynamicsWindow(std::string& sRobotFilename, Qt::WFlags flags = 0);
    ~SimDynamicsWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();
    void buildVisualization();
    void actuation();

    void loadButton();

    void selectRobotNode(int n);
    void jointValueChanged(int n);
    void fixedTimeStepChanged(int n);
    void updateTimerChanged(int n);
    void updateAntiAliasing(int n);
    void comVisu();
    void updateJointInfo();

    void startStopEngine();
    void stepEngine();

    void checkBoxFixedTimeStep();

	void addObject();
protected:
    bool loadRobot(std::string robotFilename);
    void setupUI();
    void updateJoints();

    void stopCB();

    void updateContactVisu();
    void updateComVisu();

    SimDynamics::DynamicsWorldPtr dynamicsWorld;
    SimDynamics::DynamicsRobotPtr dynamicsRobot;
    SimDynamics::DynamicsObjectPtr dynamicsObject;
    SimDynamics::DynamicsObjectPtr dynamicsObject2;
    std::vector<SimDynamics::DynamicsObjectPtr> dynamicsObjects;

    Ui::MainWindowBulletViewer UI;

    SoSeparator* sceneSep;
    SoSeparator* comSep;
    SoSeparator* contactsSep;
    SoSeparator* forceSep;

    SimDynamics::BulletCoinQtViewerPtr viewer;

    VirtualRobot::RobotPtr robot;

    // beside the viewer cb we need also a callback to update joint info
    static void timerCB(void* data, SoSensor* sensor);

    SoTimerSensor* timerSensor;

    std::vector<VirtualRobot::RobotNodeRevolutePtr> robotNodes;

    std::map< VirtualRobot::RobotNodePtr, SoSeparator* > comVisuMap;

    bool useColModel;
};

#endif // __SimDynamics_WINDOW_H_
