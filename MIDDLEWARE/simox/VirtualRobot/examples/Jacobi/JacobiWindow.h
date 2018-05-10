
#ifndef __Jacobi_WINDOW_H_
#define __Jacobi_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
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

#include "ui_Jacobi.h"

class JacobiWindow : public QMainWindow
{
    Q_OBJECT
public:
    JacobiWindow(std::string& sRobotFilename, Qt::WFlags flags = 0);
    ~JacobiWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

    SoQtExaminerViewer* getExaminerViewer()
    {
        return exViewer;
    };

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();
    void collisionModel();
    void loadRobot();
    void selectKC(int nr);
    void sliderReleased();
    void sliderPressed();

    void box2TCP();
    void jacobiTest();
    void jacobiTest2();
    void jacobiTestBi();

protected:
    void setupUI();
    QString formatString(const char* s, float f);
    void updateKCBox();

    void updatBoxPos(float x, float y, float z, float a, float b, float g);
    void updatBox2Pos(float x, float y, float z, float a, float b, float g);
    void updatBoxBiPos(float x, float y, float z, float a, float b, float g);

    static void updateCB(void* data, SoSensor* sensor);


    Ui::MainWindowJacobiDemo UI;
    SoQtExaminerViewer* exViewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* boxSep;
    SoSeparator* box2Sep;
    SoSeparator* box3Sep;

    VirtualRobot::RobotPtr robot;
    std::string robotFilename;
    VirtualRobot::RobotNodePtr tcp;
    VirtualRobot::RobotNodePtr tcp2;
    VirtualRobot::RobotNodePtr elbow;
    VirtualRobot::RobotNodeSetPtr kc;
    std::vector<VirtualRobot::RobotNodeSetPtr> kinChains;

    VirtualRobot::ObstaclePtr box;
    VirtualRobot::ObstaclePtr box2;
    VirtualRobot::ObstaclePtr box3;

    bool useColModel;
};

#endif // __Jacobi_WINDOW_H_
