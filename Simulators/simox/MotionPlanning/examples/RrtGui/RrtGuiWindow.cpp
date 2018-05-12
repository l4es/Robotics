
#include "RrtGuiWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/ManipulationObject.h"
#include "VirtualRobot/Grasping/Grasp.h"
#include "VirtualRobot/IK/GenericIKSolver.h"
#include "VirtualRobot/Grasping/GraspSet.h"
#include "VirtualRobot/CollisionDetection/CDManager.h"
#include "VirtualRobot/XML/ObjectIO.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/Planner/Rrt.h"
#include "MotionPlanning/Planner/BiRrt.h"
#include "MotionPlanning/PostProcessing/ShortcutProcessor.h"
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
#include <QFileDialog>
#include <Eigen/Geometry>
#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/nodes/SoMatrixTransform.h>

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 200.0f;

RrtGuiWindow::RrtGuiWindow(const std::string& sceneFile, const std::string& sConf, const std::string& gConf,
                           const std::string& rns, const std::string& colModelRob1, const std::string& colModelRob2,  const std::string& colModelEnv, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->sceneFile = sceneFile;

    allSep = new SoSeparator;
    allSep->ref();
    sceneFileSep = new SoSeparator;
    startGoalSep = new SoSeparator;
    rrtSep = new SoSeparator;

    allSep->addChild(sceneFileSep);
    allSep->addChild(startGoalSep);
    allSep->addChild(rrtSep);

    setupUI();

    loadScene();

    selectRNS(rns);
    selectStart(sConf);
    selectGoal(gConf);

    selectColModelRobA(colModelRob1);
    selectColModelRobB(colModelRob2);
    selectColModelEnv(colModelEnv);

    if (sConf != "")
    {
        UI.comboBoxStart->setEnabled(false);
    }

    if (gConf != "")
    {
        UI.comboBoxGoal->setEnabled(false);
    }

    if (rns != "")
    {
        UI.comboBoxRNS->setEnabled(false);
    }

    if (colModelRob1 != "")
    {
        UI.comboBoxColModelRobot->setEnabled(false);
    }

    if (colModelRob2 != "")
    {
        UI.comboBoxColModelRobotStatic->setEnabled(false);
    }

    //if (colModelEnv!="")
    //  UI.comboBoxColModelEnv->setEnabled(false);

    viewer->viewAll();

    SoSensorManager* sensor_mgr = SoDB::getSensorManager();
    SoTimerSensor* timer = new SoTimerSensor(timerCB, this);
    timer->setInterval(SbTime(TIMER_MS / 1000.0f));
    sensor_mgr->insertTimerSensor(timer);
}


RrtGuiWindow::~RrtGuiWindow()
{
    allSep->unref();
}


void RrtGuiWindow::timerCB(void* data, SoSensor* sensor)
{
    RrtGuiWindow* ikWindow = static_cast<RrtGuiWindow*>(data);
    ikWindow->redraw();
}


void RrtGuiWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);

    viewer->setAntialiasing(true, 4);

    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(allSep);
    viewer->viewAll();

    QString q1("Rrt Extend");
    QString q2("Rrt Connect");
    QString q3("BiRrt Ext/Ext");
    QString q4("BiRrt Ext/Con");
    QString q5("BiRrt Con/Ext");
    QString q6("BiRrt Con/Con");
    UI.comboBoxRRT->addItem(q1);
    UI.comboBoxRRT->addItem(q2);
    UI.comboBoxRRT->addItem(q3);
    UI.comboBoxRRT->addItem(q4);
    UI.comboBoxRRT->addItem(q5);
    UI.comboBoxRRT->addItem(q6);
    UI.comboBoxRRT->setCurrentIndex(3);

    UI.radioButtonSolution->setChecked(true);

    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadSceneWindow()));
    connect(UI.checkBoxShowSolution, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowSolutionOpti, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxShowRRT, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxStartGoal, SIGNAL(clicked()), this, SLOT(buildVisu()));
    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
    connect(UI.horizontalSliderPos, SIGNAL(sliderMoved(int)), this, SLOT(sliderSolution(int)));
    connect(UI.radioButtonSolution, SIGNAL(clicked()), this, SLOT(solutionSelected()));
    connect(UI.radioButtonSolutionOpti, SIGNAL(clicked()), this, SLOT(solutionSelected()));

    connect(UI.comboBoxStart, SIGNAL(activated(int)), this, SLOT(selectStart(int)));
    connect(UI.comboBoxGoal, SIGNAL(activated(int)), this, SLOT(selectGoal(int)));
    connect(UI.comboBoxRNS, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxColModelRobot, SIGNAL(activated(int)), this, SLOT(selectColModelRobA(int)));
    connect(UI.comboBoxColModelRobotStatic, SIGNAL(activated(int)), this, SLOT(selectColModelRobB(int)));
    connect(UI.comboBoxColModelEnv, SIGNAL(activated(int)), this, SLOT(selectColModelEnv(int)));


}




void RrtGuiWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void RrtGuiWindow::buildVisu()
{
    sceneFileSep->removeAllChildren();

    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (scene)
    {
        visualization = scene->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = NULL;

        if (visualization)
        {
            visualisationNode = visualization->getCoinVisualization();
        }

        if (visualisationNode)
        {
            sceneFileSep->addChild(visualisationNode);
        }
    }

    startGoalSep->removeAllChildren();

    if (UI.checkBoxStartGoal->isChecked())
    {
        if (robotStart)
        {
            SoNode* st = CoinVisualizationFactory::getCoinVisualization(robotStart, colModel);

            if (st)
            {
                startGoalSep->addChild(st);
            }
        }

        if (robotGoal)
        {
            SoNode* go = CoinVisualizationFactory::getCoinVisualization(robotGoal, colModel);

            if (go)
            {
                startGoalSep->addChild(go);
            }
        }
    }

    buildRRTVisu();

    redraw();
}

int RrtGuiWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void RrtGuiWindow::quit()
{
    std::cout << "RrtGuiWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void RrtGuiWindow::loadSceneWindow()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Scene File"), QString(), tr("XML Files (*.xml)"));

    if (fi == "")
    {
        return;
    }

    sceneFile = std::string(fi.toAscii());
    loadScene();
}

void RrtGuiWindow::loadScene()
{
    this->rns.reset();
    robot.reset();
    scene = SceneIO::loadScene(sceneFile);

    if (!scene)
    {
        VR_ERROR << " no scene ..." << endl;
        return;
    }

    //SceneIO::saveScene(scene,"testSaveScene.xml");
    std::vector< RobotPtr > robots = scene->getRobots();

    if (robots.size() != 1)
    {
        VR_ERROR << "Need exactly 1 robot" << endl;
        return;
    }

    robot = robots[0];
    robotStart = robot->clone("StartConfig");
    robotGoal = robot->clone("GoalConfig");
    configs = scene->getRobotConfigs(robot);

    if (configs.size() < 2)
    {
        VR_ERROR << "Need at least 2 Robot Configurations" << endl;
        return;
    }

    UI.comboBoxGoal->clear();
    UI.comboBoxStart->clear();

    for (size_t i = 0; i < configs.size(); i++)
    {
        QString qtext = configs[i]->getName().c_str();
        UI.comboBoxStart->addItem(qtext);
        UI.comboBoxGoal->addItem(qtext);
    }

    UI.comboBoxStart->setCurrentIndex(0);
    selectStart(0);
    UI.comboBoxGoal->setCurrentIndex(1);
    selectGoal(1);

    std::vector<SceneObjectSetPtr> soss = scene->getSceneObjectSets();
    UI.comboBoxColModelEnv->clear();
    QString qtext;

    for (size_t i = 0; i < soss.size(); i++)
    {
        qtext = soss[i]->getName().c_str();
        UI.comboBoxColModelEnv->addItem(qtext);
    }

    qtext = "<none>";
    UI.comboBoxColModelEnv->addItem(qtext);

    std::vector<RobotNodeSetPtr> rnss = robot->getRobotNodeSets();
    UI.comboBoxColModelRobot->clear();
    UI.comboBoxColModelRobotStatic->clear();
    UI.comboBoxRNS->clear();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        qtext = rnss[i]->getName().c_str();
        UI.comboBoxColModelRobot->addItem(qtext);
        UI.comboBoxColModelRobotStatic->addItem(qtext);
        UI.comboBoxRNS->addItem(qtext);
    }

    qtext = "<none>";
    UI.comboBoxColModelRobot->addItem(qtext);
    UI.comboBoxColModelRobotStatic->addItem(qtext);
    robot->setThreadsafe(false);
    buildVisu();
}

void RrtGuiWindow::selectStart(const std::string& conf)
{
    for (size_t i = 0; i < configs.size(); i++)
    {
        if (configs[i]->getName() == conf)
        {
            selectStart(i);
            UI.comboBoxStart->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No configuration with name <" << conf << "> found..." << endl;
}
void RrtGuiWindow::selectGoal(const std::string& conf)
{
    for (size_t i = 0; i < configs.size(); i++)
    {
        if (configs[i]->getName() == conf)
        {
            selectGoal(i);
            UI.comboBoxGoal->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No configuration with name <" << conf << "> found..." << endl;
}

void RrtGuiWindow::selectRNS(const std::string& rns)
{
    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == rns)
        {
            selectRNS(i);
            UI.comboBoxRNS->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No rns with name <" << rns << "> found..." << endl;
}

void RrtGuiWindow::selectColModelRobA(const std::string& colModel)
{
    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelRobA(i);
            UI.comboBoxColModelRobot->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No col model set with name <" << colModel << "> found..." << endl;
}
void RrtGuiWindow::selectColModelRobB(const std::string& colModel)
{
    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelRobB(i);
            UI.comboBoxColModelRobotStatic->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No col model set with name <" << colModel << "> found..." << endl;
}
void RrtGuiWindow::selectColModelEnv(const std::string& colModel)
{
    if (!scene)
    {
        return;
    }

    std::vector< SceneObjectSetPtr > rnss = scene->getSceneObjectSets();

    for (size_t i = 0; i < rnss.size(); i++)
    {
        if (rnss[i]->getName() == colModel)
        {
            selectColModelEnv(i);
            UI.comboBoxColModelEnv->setCurrentIndex(i);
            return;
        }
    }

    VR_ERROR << "No scene object set with name <" << colModel << "> found..." << endl;
}

void RrtGuiWindow::selectStart(int nr)
{
    if (nr < 0 || nr >= (int)configs.size())
    {
        return;
    }

    if (robotStart)
    {
        robotStart->setJointValues(configs[nr]);
    }

    if (robot)
    {
        robot->setJointValues(configs[nr]);
    }

    //configs[nr]->setJointValues();
    if (rns)
    {
        rns->getJointValues(startConfig);
    }
}

void RrtGuiWindow::selectGoal(int nr)
{
    if (nr < 0 || nr >= (int)configs.size())
    {
        return;
    }

    if (robotGoal)
    {
        robotGoal->setJointValues(configs[nr]);
    }

    robot->setJointValues(configs[nr]);

    if (rns)
    {
        rns->getJointValues(goalConfig);
    }
}
void RrtGuiWindow::selectRNS(int nr)
{
    this->rns.reset();

    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->rns = rnss[nr];
}
void RrtGuiWindow::selectColModelRobA(int nr)
{
    colModelRobA.reset();

    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelRobA = robot->getRobotNodeSet(rnss[nr]->getName());
}
void RrtGuiWindow::selectColModelRobB(int nr)
{
    colModelRobB.reset();

    if (!robot)
    {
        return;
    }

    std::vector< RobotNodeSetPtr > rnss = robot->getRobotNodeSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelRobB = robot->getRobotNodeSet(rnss[nr]->getName());
}
void RrtGuiWindow::selectColModelEnv(int nr)
{
    colModelEnv.reset();

    if (!scene)
    {
        return;
    }

    std::vector< SceneObjectSetPtr > rnss = scene->getSceneObjectSets();

    if (nr < 0 || nr >= (int)rnss.size())
    {
        return;
    }

    this->colModelEnv = scene->getSceneObjectSet(rnss[nr]->getName());
}
void RrtGuiWindow::buildRRTVisu()
{
    rrtSep->removeAllChildren();

    if (!cspace || !robot)
    {
        return;
    }

    boost::shared_ptr<Saba::CoinRrtWorkspaceVisualization> w(new Saba::CoinRrtWorkspaceVisualization(robot, cspace, rns->getTCP()->getName()));

    if (UI.checkBoxShowRRT->isChecked())
    {
        if (tree)
        {
            w->addTree(tree);
        }

        if (tree2)
        {
            w->addTree(tree2);
        }
    }

    if (UI.checkBoxShowSolution->isChecked() && solution)
    {
        w->addCSpacePath(solution);
    }

    if (UI.checkBoxShowSolutionOpti->isChecked() && solutionOptimized)
    {
        w->addCSpacePath(solutionOptimized, Saba::CoinRrtWorkspaceVisualization::eGreen);
    }

    //w->addConfiguration(startConfig,Saba::CoinRrtWorkspaceVisualization::eGreen,3.0f);
    //w->addConfiguration(goalConfig,Saba::CoinRrtWorkspaceVisualization::eRed,3.0f);
    SoSeparator* sol = w->getCoinVisualization();
    rrtSep->addChild(sol);
}

void RrtGuiWindow::plan()
{
    if (!robot || !rns)
    {
        return;
    }

    // setup collision detection
    CDManagerPtr cdm(new CDManager());

    if (colModelRobA)
    {
        cdm->addCollisionModel(colModelRobA);
    }

    if (colModelRobB)
    {
        cdm->addCollisionModel(colModelRobB);
    }

    if (colModelEnv)
    {
        cdm->addCollisionModel(colModelEnv);
    }

	cspace.reset(new Saba::CSpaceSampled(robot, cdm, rns, 1000000));
    float sampl = (float)UI.doubleSpinBoxCSpaceSampling->value();
    float samplDCD = (float)UI.doubleSpinBoxColChecking->value();
    cspace->setSamplingSize(sampl);
    cspace->setSamplingSizeDCD(samplDCD);
    Saba::Rrt::RrtMethod mode;
    Saba::Rrt::RrtMethod mode2;
    bool planOk = false;
    Saba::RrtPtr mp;
    Saba::BiRrtPtr mpBi;
    bool biRRT = false;

    if (UI.comboBoxRRT->currentIndex() == 0 || UI.comboBoxRRT->currentIndex() == 1)
    {
        if (UI.comboBoxRRT->currentIndex() == 0)
        {
            mode = Saba::Rrt::eExtend;
        }
        else
        {
            mode = Saba::Rrt::eConnect;
        }

        Saba::RrtPtr rrt(new Saba::Rrt(cspace, mode));
        mp = rrt;
    }
    else
    {
        biRRT = true;

        if (UI.comboBoxRRT->currentIndex() == 2)
        {
            mode = Saba::Rrt::eExtend;
            mode2 = Saba::Rrt::eExtend;
        }
        else if (UI.comboBoxRRT->currentIndex() == 3)
        {
            mode = Saba::Rrt::eExtend;
            mode2 = Saba::Rrt::eConnect;
        }
        else if (UI.comboBoxRRT->currentIndex() == 4)
        {
            mode = Saba::Rrt::eConnect;
            mode2 = Saba::Rrt::eExtend;
        }
        else
        {
            mode = Saba::Rrt::eConnect;
            mode2 = Saba::Rrt::eConnect;
        }

        Saba::BiRrtPtr rrt(new Saba::BiRrt(cspace, mode, mode2));
        mp = rrt;
        mpBi = rrt;
    }

    mp->setStart(startConfig);
    mp->setGoal(goalConfig);

    bool planOK = mp->plan();

    if (planOK)
    {
        VR_INFO << " Planning succeeded " << endl;
        solution = mp->getSolution();
        Saba::ShortcutProcessorPtr postProcessing(new Saba::ShortcutProcessor(solution, cspace, false));
        solutionOptimized = postProcessing->optimize(100);
        tree = mp->getTree();

        if (biRRT)
        {
            tree2 = mpBi->getTree2();
        }
        else
        {
            tree2.reset();
        }

    }
    else
    {
        VR_INFO << " Planning failed" << endl;
    }

    sliderSolution(1000);

    buildVisu();
}

void RrtGuiWindow::colModel()
{
    buildVisu();
}
void RrtGuiWindow::solutionSelected()
{
    sliderSolution(UI.horizontalSliderPos->sliderPosition());
}
void RrtGuiWindow::sliderSolution(int pos)
{
    if (!solution)
    {
        return;
    }

    Saba::CSpacePathPtr s = solution;

    if (UI.radioButtonSolutionOpti->isChecked() && solutionOptimized)
    {
        s = solutionOptimized;
    }

    float p = (float)pos / 1000.0f;
    Eigen::VectorXf iPos;
    s->interpolate(p, iPos);
    robot->setJointValues(rns, iPos);
    redraw();
}

void RrtGuiWindow::redraw()
{
    viewer->scheduleRedraw();
    UI.frameViewer->update();
    viewer->scheduleRedraw();
    this->update();
    viewer->scheduleRedraw();
}


