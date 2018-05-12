
#include "MTPlanningWindow.h"


#include <vector>
#include <iostream>
#include <qlayout.h>
#include <qlabel.h>
#include <qpixmap.h>
#include <qprogressbar.h>
#include <qcheckbox.h>
#include <qlcdnumber.h>
#include <qslider.h>
#include <qimage.h>
#include <qgl.h>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoMatrixTransform.h>

#include <sstream>

using namespace std;

//Globel variables
float TIMER_MS = 30.0f;
bool runtimeDisplayed = false;
bool optiTimeDisplayed = false;

MTPlanningWindow::MTPlanningWindow(Qt::WFlags flags)
    : QMainWindow(NULL)
{
    //resize(1100, 768);

    graspObjectSep = NULL;
    robotSep = NULL;

    scene = NULL;

    scene = new MTPlanningScenery();
    sceneSep = scene->getScene();
    setupLayoutMTPlanning();


    SoSensorManager* sensor_mgr = SoDB::getSensorManager();

    SoTimerSensor* timer1 = new SoTimerSensor(timerCBPlanning, this);
    SoTimerSensor* timer2 = new SoTimerSensor(timerCBOptimize, this);
    timer1->setInterval(SbTime(TIMER_MS / 1000.0f));
    timer2->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timer1);
    sensor_mgr->insertTimerSensor(timer2);
}


MTPlanningWindow::~MTPlanningWindow()
{
    if (scene != NULL)
    {
        delete scene;
    }
}


void MTPlanningWindow::timerCBPlanning(void* data, SoSensor* sensor)
{
    MTPlanningWindow* mtWindow = static_cast<MTPlanningWindow*>(data);
    mtWindow->scene->checkPlanningThreads();
    int nThreadsWorking = 0;
    int nThreadsIdle = 0;
    mtWindow->scene->getThreadCount(nThreadsWorking, nThreadsIdle);
    QString sText;
    sText = "Threads_Working: " + QString::number(nThreadsWorking);
    mtWindow->UI.labelThreads->setText(sText);
    sText = "Threads_Idle: " + QString::number(nThreadsIdle);
    mtWindow->UI.labelThreadsIdle->setText(sText);

    if (!runtimeDisplayed && mtWindow->scene->getPlannersStarted() && nThreadsWorking == 0)
    {
        mtWindow->endTime = clock();
        double runtime = (double)(mtWindow->endTime - mtWindow->startTime) / CLOCKS_PER_SEC;
        sText = "Runtime: " + QString::number(runtime) + "s";
        mtWindow->UI.labelRuntime->setText(sText);
        cout << "Runtime = " << runtime << endl;
        runtimeDisplayed = true;
    }
}


void MTPlanningWindow::timerCBOptimize(void* data, SoSensor* sensor)
{
    MTPlanningWindow* mtWindow = static_cast<MTPlanningWindow*>(data);
    mtWindow->scene->checkOptimizeThreads();
    int nWorking = 0;
    int nIdle = 0;
    mtWindow->scene->getOptimizeThreadCount(nWorking, nIdle);
    QString sText;
    sText = "Opti_Threads_Working: " + QString::number(nWorking);
    mtWindow->UI.labelOptiWork->setText(sText);
    sText = "Opti_Threads_Idle: " + QString::number(nIdle);
    mtWindow->UI.labelOptiIdle->setText(sText);

    if (!optiTimeDisplayed && mtWindow->scene->getOptimizeStarted() && (nWorking == 0))
    {
        mtWindow->optiEndTime = clock();
        double runtime = (double)(mtWindow->optiEndTime - mtWindow->optiStartTime) / CLOCKS_PER_SEC;
        cout << "Runtime:" << runtime << endl;
        sText = "Optimizing Time: " + QString::number(runtime) + "s";
        mtWindow->UI.labelOptiTime->setText(sText);
        cout << "Optimizing Time = " << runtime << endl;
        optiTimeDisplayed = true;
    }
}


void MTPlanningWindow::setupLayoutMTPlanning()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
    viewer->setAntialiasing(true, 4);
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_LAYERS_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph((SoNode*)sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonBuild, SIGNAL(clicked()), this, SLOT(buildScene()));
    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(reset()));

    UI.comboBoxColChecking->addItem(QString("Singleton Col Checker"));
    UI.comboBoxColChecking->addItem(QString("Multiple Col Checker Instances"));
    UI.comboBoxColChecking->setCurrentIndex(1);
    connect(UI.comboBoxColChecking, SIGNAL(activated(int)), this, SLOT(selectColCheckerComboBoxChanged(int)));
    connect(UI.pushButtonAdd, SIGNAL(clicked()), this, SLOT(addThread()));
    connect(UI.pushButtonPlanning, SIGNAL(clicked()), this, SLOT(startThreads()));
    connect(UI.pushButtonPlanningStop, SIGNAL(clicked()), this, SLOT(stopThreads()));
    connect(UI.pushButtonPost, SIGNAL(clicked()), this, SLOT(startOptimize()));
    connect(UI.pushButtonPostStop, SIGNAL(clicked()), this, SLOT(stopOptimize()));
}

void MTPlanningWindow::selectColCheckerComboBoxChanged(int value)
{
    reset();
}

void MTPlanningWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


int MTPlanningWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void MTPlanningWindow::quit()
{
    std::cout << "MTPlanningWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}


void MTPlanningWindow::reset()
{
    std::cout << "MTPlanningWindow: Reset" << std::endl;
    runtimeDisplayed = false;
    optiTimeDisplayed = false;
    scene->reset();
    viewer->viewAll();
}

void MTPlanningWindow::buildScene()
{
    std::cout << "MTPlanningWindow: buildScene " << std::endl;
    scene->reset();
    scene->buildScene();
    viewer->viewAll();
}

void MTPlanningWindow::addThread()
{
    int n = UI.spinBoxThreads->value();
    std::cout << "MTPlanningWindow: addThread " << n << std::endl;
    bool bMultipleThreads = false;

    if (UI.comboBoxColChecking->currentIndex() == 1)
    {
        bMultipleThreads = true;
    }

    int thr = scene->getThreads();

    for (int i = 0; i < n; i++)
    {
        scene->buildPlanningThread(bMultipleThreads, thr + i);
    }
}


void MTPlanningWindow::startThreads()
{
    std::cout << "MTPlanningWindow: startThreads " << std::endl;
    this->startTime = clock();
    scene->startPlanning();
    runtimeDisplayed = false;
}


void MTPlanningWindow::stopThreads()
{
    std::cout << "MTPlanningWindow: stopThreads " << std::endl;
    scene->stopPlanning();
}

void MTPlanningWindow::startOptimize()
{
    std::cout << "MTPlanningWindow: startOptimize " << std::endl;
    this->optiStartTime = clock();
    scene->startOptimizing();
    optiTimeDisplayed = false;
}

void MTPlanningWindow::stopOptimize()
{
    std::cout << "MTPlanningWindow: stopOptimize " << std::endl;
    scene->stopOptimizing();
}

/////////////////////////////////////////////////////////////////
// Sequential Planing
/*void MTPlanningWindow::plan()
{
    clock_t start, end;
    start = clock();
    unsigned int i = 0;
    for(; i < NUMBER_OF_PLANNING; i++)
    {
        scene->plan(i);
    }
    end = clock();
    double runtime = (double)(end - start) / CLOCKS_PER_SEC;
    QString sText;
    sText = "Plan Time: " + QString::number(runtime) + "s";
    sQRuntimeLabel->setText(sText);
    cout << "The total plantime is " << runtime << " ." << endl;
    cout << "The number of the plannings is " << i << " ." << endl;
}

void MTPlanningWindow::optimizeSolution()
{
    clock_t start, end;
    start = clock();
    unsigned int i = 0;
    for(; i < NUMBER_OF_PLANNING; i++)
    {
        scene->optimizeSolution(i);
    }
    end = clock();
    double runtime = (double)(end - start) / CLOCKS_PER_SEC;
    QString sText;
    sText = "optimizing Time: " + QString::number(runtime) + "s";
    sQOptimizeTimeLabel->setText(sText);
    cout << "The total optimization time is " << runtime << " ." << endl;
    cout << "The number of the optimizations is " << i << " ." << endl;
}*/
