
#ifndef __MTPlanning_WINDOW_H_
#define __MTPlanning_WINDOW_H_

#include "MotionPlanning/Saba.h"

#include "MTPlanningScenery.h"
#include <VirtualRobot/Robot.h>

#include <qobject.h>
#include <qmainwindow.h>
#include <qpushbutton.h>
#include <qtextedit.h>
#include <qcombobox.h>
#include <qprogressbar.h>
#include <qlabel.h>
#include <qslider.h>
#include <qcheckbox.h>
#include <string.h>
#include <time.h>

#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/SoOffscreenRenderer.h>


#include "ui_MTPlanning.h"

#define NUMBER_OF_PLANNING 30

class MTPlanningWindow : public QMainWindow
{
    Q_OBJECT

public:
    MTPlanningWindow(Qt::WFlags flags = 0);
    ~MTPlanningWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    //void loadRobot();
    void buildScene();
    void addThread();
    void startThreads();
    void stopThreads();
    void startOptimize();
    void stopOptimize();
    void reset();
    void selectColCheckerComboBoxChanged(int value);

    SoQtExaminerViewer* getExaminerViewer()
    {
        return viewer;
    };

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // Sequential Planing
    //void plan();
    //void optimizeSolution();

protected:
    //void setupMenus(); /*!< Setup the menus. */
    void setupLayoutMTPlanning(); /*!< Create the contents of the window. */

    static void timerCBPlanning(void* data, SoSensor* sensor);
    static void timerCBOptimize(void* data, SoSensor* sensor);

    Ui::MainWindowMTPlanning UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* robotSep;
    SoSeparator* graspObjectSep;

    MTPlanningScenery* scene;

    clock_t startTime;
    clock_t endTime;
    clock_t optiStartTime;
    clock_t optiEndTime;

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // Sequential Planing
    /*QPushButton *loadRobotButton;
    QPushButton *setConfigButton;
    QPushButton *planButton;
    QPushButton *optiShowButton;*/
};

#endif // __MTPlanning_WINDOW_H_
