
#ifndef __IKRRT_WINDOW_H_
#define __IKRRT_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>

#include "MotionPlanning/Saba.h"
#include "MotionPlanning/CSpace/CSpacePath.h"

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

#include "ui_IKRRT.h"

class IKRRTWindow : public QMainWindow
{
    Q_OBJECT
public:
    IKRRTWindow(std::string& sceneFile, std::string& reachFile, std::string& rns, std::string& eef, std::string& colModel, std::string& colModelRob, Qt::WFlags flags = 0);
    ~IKRRTWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    void redraw();
public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();


    void closeEEF();
    void openEEF();
    void searchIK();

    void colModel();

    void sliderReleased_ObjectX();
    void sliderReleased_ObjectY();
    void sliderReleased_ObjectZ();
    void sliderReleased_ObjectA();
    void sliderReleased_ObjectB();
    void sliderReleased_ObjectG();
    void sliderSolution(int pos);

    void buildVisu();

    void showCoordSystem();
    void reachVisu();

    void planIKRRT();

    void playAndSave();

protected:

    void loadScene();
    void loadReach();

    void setupUI();
    QString formatString(const char* s, float f);

    void buildGraspSetVisu();

    void buildRRTVisu();

    void updateObject(float x[6]);

    static void timerCB(void* data, SoSensor* sensor);
    void buildRrtVisu();
    void saveScreenshot();
    Ui::MainWindowIKRRT UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* objectSep;
    SoSeparator* graspsSep;
    SoSeparator* reachableGraspsSep;
    SoSeparator* reachabilitySep;
    SoSeparator* obstaclesSep;
    SoSeparator* rrtSep;

    VirtualRobot::RobotPtr robot;
    std::vector< VirtualRobot::ObstaclePtr > obstacles;
    VirtualRobot::ManipulationObjectPtr object;
    VirtualRobot::ReachabilityPtr reachSpace;

    VirtualRobot::EndEffectorPtr eef;
    Saba::CSpaceSampledPtr cspace;
    Eigen::VectorXf startConfig;

    VirtualRobot::GraspSetPtr graspSet;
    VirtualRobot::RobotNodeSetPtr rns;

    std::string sceneFile;
    std::string reachFile;
    std::string eefName;
    std::string rnsName;
    std::string colModelName;
    std::string colModelNameRob;

    Saba::CSpacePathPtr solution;
    Saba::CSpacePathPtr solutionOptimized;
    Saba::CSpaceTreePtr tree;
    Saba::CSpaceTreePtr tree2;

    bool playbackMode;
    int playCounter;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot;
    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;
};

#endif // __IKRRT_WINDOW_H_
