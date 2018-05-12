#ifndef __GraspRrt_WINDOW_H__
#define __GraspRrt_WINDOW_H__

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>

#include <GraspPlanning/GraspStudio.h>
#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>

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

#include "ui_GraspRrt.h"

class GraspRrtWindow : public QMainWindow
{
    Q_OBJECT
public:
    GraspRrtWindow(const std::string& sceneFile, const std::string& sConf, const std::string& gConf,
                   const std::string& rns, const std::string& rnsB, const std::string& eefName, const std::string& eefNameB,
                   const std::string& colModelRob1, const std::string& colModelRob1B, const std::string& colModelRob2, const std::string& colModelRob2B,
                   const std::string& colModelEnv, Qt::WFlags flags = 0);
    ~GraspRrtWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    void redraw();
public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void loadSceneWindow();

    void selectStart(int nr);
    void selectTargetObject(int nr);
    void selectRNS(int nr);
    void selectEEF(int nr);

    void selectColModelRobA(int nr);
    void selectColModelRobB(int nr);
    void selectColModelEnv(int nr);

    void colModel();
    void solutionSelected();

    void sliderSolution(int pos);

    void buildVisu();

    void plan();

    void testGraspPose();

    void openEEF();
    void closeEEF();

protected:

    struct planSet
    {
        std::string eef;
        std::string rns;
        std::string colModelRob1;
        std::string colModelRob2;
        std::string colModelEnv;
    };

    planSet planSetA, planSetB;


    void loadScene();

    void setupUI();
    QString formatString(const char* s, float f);
    void buildRRTVisu();
    void selectStart(const std::string& conf);
    void selectTargetObject(const std::string& conf);
    void selectRNS(const std::string& rns);
    void selectEEF(const std::string& eefName);

    static void timerCB(void* data, SoSensor* sensor);
    void buildRrtVisu();
    void selectColModelRobA(const std::string& colModel);
    void selectColModelRobB(const std::string& colModel);
    void selectColModelEnv(const std::string& colModel);
    void selectPlanSet(int nr);

    void testInit();

    Ui::MainWindowGraspRRTDemo UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* allSep;
    SoSeparator* sceneFileSep;
    SoSeparator* graspsSep;
    SoSeparator* rrtSep;

    VirtualRobot::RobotPtr robot;
    //VirtualRobot::RobotPtr robotStart;
    //VirtualRobot::RobotPtr robotGoal;

    Saba::CSpaceSampledPtr cspace;
    Eigen::VectorXf startConfig;
    Eigen::VectorXf goalConfig;

    VirtualRobot::RobotNodeSetPtr rns;
    VirtualRobot::EndEffectorPtr eef;
    VirtualRobot::SceneObjectSetPtr colModelRobA;
    VirtualRobot::SceneObjectSetPtr colModelRobB;
    VirtualRobot::SceneObjectSetPtr colModelEnv;

    std::vector< VirtualRobot::RobotConfigPtr > configs;
    std::vector< VirtualRobot::ObstaclePtr > obstacles;
    VirtualRobot::ObstaclePtr targetObject;

    std::vector<VirtualRobot::GraspPtr> grasps;

    std::string sceneFile;
    VirtualRobot::ScenePtr scene;

    Saba::CSpacePathPtr solution;
    Saba::CSpacePathPtr solutionOptimized;
    Saba::CSpaceTreePtr tree;
    GraspStudio::GraspQualityMeasureWrenchSpacePtr graspQuality;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualization;

    Saba::GraspRrtPtr test_graspRrt;
    Saba::CSpaceSampledPtr test_cspace;
};

#endif // __GraspRrt_WINDOW_H__
