
#include "JacobiWindow.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/IK/DifferentialIK.h"

#include <time.h>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

JacobiWindow::JacobiWindow(std::string& sRobotFilename, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);

    useColModel = false;
    robotFilename = sRobotFilename;
    sceneSep = new SoSeparator();
    sceneSep->ref();
    robotSep = new SoSeparator();
    sceneSep->addChild(robotSep);
    setupUI();

    loadRobot();

    box = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    boxSep = new SoSeparator();
    boxSep->addChild(CoinVisualization(box->getVisualization()).getCoinVisualization());
    sceneSep->addChild(boxSep);

    box2 = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    box2Sep = new SoSeparator();
    box2Sep->addChild(CoinVisualization(box2->getVisualization()).getCoinVisualization());
    sceneSep->addChild(box2Sep);

    box3 = Obstacle::createBox(30.0f, 30.0f, 30.0f);
    box3Sep = new SoSeparator();
    box3Sep->addChild(CoinVisualization(box3->getVisualization()).getCoinVisualization());
    sceneSep->addChild(box3Sep);

    box2TCP();


    exViewer->viewAll();
}


JacobiWindow::~JacobiWindow()
{
    sceneSep->unref();
}

void JacobiWindow::setupUI()
{
    UI.setupUi(this);
    exViewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    exViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    exViewer->setAccumulationBuffer(true);
#ifdef WIN32

#ifndef _DEBUG
    exViewer->setAntialiasing(true, 4);
#endif

#endif

    exViewer->setTransparencyType(SoGLRenderAction::BLEND);
    exViewer->setFeedbackVisibility(true);
    exViewer->setSceneGraph(sceneSep);
    exViewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadRobot()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.comboBoxKC, SIGNAL(activated(int)), this, SLOT(selectKC(int)));

    connect(UI.pushButtonBox2TCP, SIGNAL(clicked()), this, SLOT(box2TCP()));
    connect(UI.pushButtonJacobiTest, SIGNAL(clicked()), this, SLOT(jacobiTest()));
    connect(UI.pushButtonJacobiTest_2, SIGNAL(clicked()), this, SLOT(jacobiTestBi()));
    connect(UI.pushButtonJacobiTest_3, SIGNAL(clicked()), this, SLOT(jacobiTest2()));


    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderA, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderB, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderG, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    connect(UI.horizontalSliderX_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderY_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderZ_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderA_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderB_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderG_2, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    connect(UI.horizontalSliderX_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderY_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderZ_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderA_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderB_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderG_3, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    // a method, that is called by a timer, is allowed to update the IV models without disturbing the render loop
    SoTimerSensor* timer = new SoTimerSensor((SoSensorCB*)(&updateCB), this);
    SbTime interval(1.0 / 30);
    timer->setInterval(interval);
    timer->schedule();
}

void JacobiWindow::updateCB(void* data, SoSensor* sensor)
{
    if (!data)
    {
        return;
    }

    JacobiWindow* w = (JacobiWindow*)(data);

    if (!w)
    {
        return;
    }

    float x = w->UI.horizontalSliderX->value();
    float y = w->UI.horizontalSliderY->value();
    float z = w->UI.horizontalSliderZ->value();
    float a = w->UI.horizontalSliderA->value();
    float b = w->UI.horizontalSliderB->value();
    float g = w->UI.horizontalSliderG->value();

    if (x != 0 || y != 0 || z != 0 || a != 0 || b != 0 || g != 0)
    {
        w->updatBoxPos(x / 100.0f, y / 100.0f, z / 100.0f, a / 2000.0f, b / 2000.0f, g / 2000.0f);
    }

    x = w->UI.horizontalSliderX_2->value();
    y = w->UI.horizontalSliderY_2->value();
    z = w->UI.horizontalSliderZ_2->value();
    a = w->UI.horizontalSliderA_2->value();
    b = w->UI.horizontalSliderB_2->value();
    g = w->UI.horizontalSliderG_2->value();

    if (x != 0 || y != 0 || z != 0 || a != 0 || b != 0 || g != 0)
    {
        w->updatBox2Pos(x / 100.0f, y / 100.0f, z / 100.0f, a / 2000.0f, b / 2000.0f, g / 2000.0f);
    }

    x = w->UI.horizontalSliderX_3->value();
    y = w->UI.horizontalSliderY_3->value();
    z = w->UI.horizontalSliderZ_3->value();
    a = w->UI.horizontalSliderA_3->value();
    b = w->UI.horizontalSliderB_3->value();
    g = w->UI.horizontalSliderG_3->value();

    if (x != 0 || y != 0 || z != 0 || a != 0 || b != 0 || g != 0)
    {
        w->updatBoxBiPos(x / 100.0f, y / 100.0f, z / 100.0f, a / 2000.0f, b / 2000.0f, g / 2000.0f);
    }
}

void JacobiWindow::updatBoxPos(float x, float y, float z, float a, float b, float g)
{
    if (!box)
    {
        return;
    }

    Eigen::Matrix4f m = box->getGlobalPose();
    Eigen::Matrix4f mR;
    MathTools::rpy2eigen4f(a, b, g, mR);
    mR = m * mR;
    mR(0, 3) = m(0, 3) + x;
    mR(1, 3) = m(1, 3) + y;
    mR(2, 3) = m(2, 3) + z;
    box->setGlobalPose(mR);
    exViewer->render();
}

void JacobiWindow::updatBoxBiPos(float x, float y, float z, float a, float b, float g)
{
    if (!box3)
    {
        return;
    }

    Eigen::Matrix4f m = box3->getGlobalPose();
    Eigen::Matrix4f mR;
    MathTools::rpy2eigen4f(a, b, g, mR);
    mR = m * mR;
    mR(0, 3) = m(0, 3) + x;
    mR(1, 3) = m(1, 3) + y;
    mR(2, 3) = m(2, 3) + z;
    box3->setGlobalPose(mR);
    exViewer->render();
}

void JacobiWindow::updatBox2Pos(float x, float y, float z, float a, float b, float g)
{
    if (!box2)
    {
        return;
    }

    Eigen::Matrix4f m = box2->getGlobalPose();
    Eigen::Matrix4f mR;
    MathTools::rpy2eigen4f(a, b, g, mR);
    mR = m * mR;
    mR(0, 3) = m(0, 3) + x;
    mR(1, 3) = m(1, 3) + y;
    mR(2, 3) = m(2, 3) + z;
    box2->setGlobalPose(mR);
    exViewer->render();
}

QString JacobiWindow::formatString(const char* s, float f)
{
    QString str1(s);

    if (f >= 0)
    {
        str1 += " ";
    }

    if (fabs(f) < 1000)
    {
        str1 += " ";
    }

    if (fabs(f) < 100)
    {
        str1 += " ";
    }

    if (fabs(f) < 10)
    {
        str1 += " ";
    }

    QString str1n;
    str1n.setNum(f, 'f', 3);
    str1 = str1 + str1n;
    return str1;
}


void JacobiWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<RobotNodePtr> rn;
    robot->getRobotNodes(rn);
    std::vector<float> jv(rn.size(), 0.0f);
    robot->setJointValues(rn, jv);

    exViewer->render();
}



void JacobiWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    robotSep->removeAllChildren();
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = useColModel ? SceneObject::Collision : SceneObject::Full;

    boost::shared_ptr<CoinVisualization> visualization = robot->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotSep->addChild(visualisationNode);
    }

    exViewer->render();
}


void JacobiWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


int JacobiWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void JacobiWindow::quit()
{
    std::cout << "JacobiWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void JacobiWindow::updateKCBox()
{
    UI.comboBoxKC->clear();

    if (!robot)
    {
        return;
    }

    std::vector<RobotNodeSetPtr> rns;
    robot->getRobotNodeSets(rns);
    kinChains.clear();

    for (unsigned int i = 0; i < rns.size(); i++)
    {
        if (rns[i]->isKinematicChain())
        {
            UI.comboBoxKC->addItem(QString(rns[i]->getName().c_str()));
            kinChains.push_back(rns[i]);
        }
    }
}

void JacobiWindow::selectKC(int nr)
{
    cout << "Selecting kinematic chain nr " << nr << endl;

    if (nr < 0 || nr >= (int)kinChains.size())
    {
        return;
    }

    kc = kinChains[nr];
    kc->print();
    QString nameQ = "TCP: ";
    tcp = kc->getTCP();

    if (tcp)
    {
        QString n(tcp->getName().c_str());
        nameQ += n;
    }

    float d = kc->getSize();
    QString qd = "Nr of joints: ";
    qd += QString::number(d);

    UI.label_TCP->setText(nameQ);
    UI.label_NrJoints->setText(qd);
    std::vector<RobotNodePtr> nodes = kc->getAllRobotNodes();
    elbow.reset();

    for (size_t i = 0; i < nodes.size(); i++)
    {
        if ((nodes[i])->getName() == std::string("Elbow L"))
        {
            elbow = nodes[i];
        }

        if ((nodes[i])->getName() == std::string("Elbow R"))
        {
            elbow = nodes[i];
        }

        if ((nodes[i])->getName() == std::string("TCP L"))
        {
            elbow = nodes[i];
        }

        if ((nodes[i])->getName() == std::string("TCP R"))
        {
            elbow = nodes[i];
        }
    }

    tcp2.reset();

    if (kc->getTCP()->getName() == std::string("TCP L"))
    {
        tcp2 = robot->getRobotNode(std::string("TCP R"));
    }

    if (kc->getTCP()->getName() == std::string("TCP R"))
    {
        tcp2 = robot->getRobotNode(std::string("TCP L"));
    }

    box2TCP();
}

void JacobiWindow::sliderReleased()
{
    UI.horizontalSliderX->setSliderPosition(0);
    UI.horizontalSliderY->setSliderPosition(0);
    UI.horizontalSliderZ->setSliderPosition(0);
    UI.horizontalSliderA->setSliderPosition(0);
    UI.horizontalSliderB->setSliderPosition(0);
    UI.horizontalSliderG->setSliderPosition(0);
    UI.horizontalSliderX_2->setSliderPosition(0);
    UI.horizontalSliderY_2->setSliderPosition(0);
    UI.horizontalSliderZ_2->setSliderPosition(0);
    UI.horizontalSliderA_2->setSliderPosition(0);
    UI.horizontalSliderB_2->setSliderPosition(0);
    UI.horizontalSliderG_2->setSliderPosition(0);
    UI.horizontalSliderX_3->setSliderPosition(0);
    UI.horizontalSliderY_3->setSliderPosition(0);
    UI.horizontalSliderZ_3->setSliderPosition(0);
    UI.horizontalSliderA_3->setSliderPosition(0);
    UI.horizontalSliderB_3->setSliderPosition(0);
    UI.horizontalSliderG_3->setSliderPosition(0);
    exViewer->render();
}


void JacobiWindow::jacobiTest()
{
    if (!kc)
    {
        return;
    }

    cout << "---- TEST JACOBI ----" << endl;
    DifferentialIKPtr j(new DifferentialIK(kc));

    Eigen::Matrix4f targetPose = box->getGlobalPose();

    j->setGoal(targetPose, RobotNodePtr(), IKSolver::All);
    j->computeSteps(0.2f, 0, 50);
    exViewer->render();

    cout << "---- END TEST JACOBI ----" << endl;
}

void JacobiWindow::jacobiTest2()
{
    if (!kc || !elbow)
    {
        return;
    }

    cout << "---- TEST JACOBI ----" << endl;
    //std::vector<RobotNodePtr> n;
    //n.push_back(tcp);
    //n.push_back(elbow);
    //RobotNodeSetPtr rns = RobotNodeSet::createRobotNodeSet(robot,std::string("jacobiTest"),n);
    DifferentialIKPtr j(new DifferentialIK(kc, RobotNodePtr()/*,rns*/));

    Eigen::Matrix4f targetPose = box->getGlobalPose();
    Eigen::Matrix4f targetPose2 = box2->getGlobalPose();

    j->setGoal(targetPose, tcp, IKSolver::Position);
    j->setGoal(targetPose2, elbow, IKSolver::Z);
    j->computeSteps(0.2f, 0, 40);
    exViewer->render();

    cout << "---- END TEST JACOBI ----" << endl;
}

void JacobiWindow::jacobiTestBi()
{
    if (!kc || !tcp || !tcp2)
    {
        return;
    }

    cout << "---- TEST JACOBI ----" << endl;
    //std::vector<RobotNodePtr> n;
    //n.push_back(tcp);
    //n.push_back(tcp2);
    //RobotNodeSetPtr rns = RobotNodeSet::createRobotNodeSet(robot,std::string("jacobiTest"),n);
    std::vector<RobotNodePtr> nBi;
    nBi.push_back(robot->getRobotNode(std::string("Shoulder 1 L")));
    nBi.push_back(robot->getRobotNode(std::string("Shoulder 1 R")));
    nBi.push_back(robot->getRobotNode(std::string("Shoulder 2 L")));
    nBi.push_back(robot->getRobotNode(std::string("Shoulder 2 R")));
    nBi.push_back(robot->getRobotNode(std::string("Underarm L")));
    nBi.push_back(robot->getRobotNode(std::string("Underarm R")));
    nBi.push_back(robot->getRobotNode(std::string("Elbow L")));
    nBi.push_back(robot->getRobotNode(std::string("Elbow R")));
    nBi.push_back(robot->getRobotNode(std::string("Upperarm L")));
    nBi.push_back(robot->getRobotNode(std::string("Upperarm R")));
    nBi.push_back(robot->getRobotNode(std::string("Wrist 1 L")));
    nBi.push_back(robot->getRobotNode(std::string("Wrist 1 R")));
    nBi.push_back(robot->getRobotNode(std::string("Wrist 2 L")));
    nBi.push_back(robot->getRobotNode(std::string("Wrist 2 R")));
    nBi.push_back(robot->getRobotNode(std::string("Hip Roll")));
    nBi.push_back(robot->getRobotNode(std::string("Hip Pitch")));
    nBi.push_back(robot->getRobotNode(std::string("Hip Yaw")));
    RobotNodeSetPtr kcBi = RobotNodeSet::createRobotNodeSet(robot, std::string("jacobiTestBi"), nBi);

    DifferentialIKPtr j(new DifferentialIK(kcBi, RobotNodePtr()/*,rns*/));

    Eigen::Matrix4f targetPose = box->getGlobalPose();
    Eigen::Matrix4f targetPose2 = box3->getGlobalPose();

    j->setGoal(targetPose, tcp, IKSolver::Position);
    j->setGoal(targetPose2, tcp2, IKSolver::Position);
    j->computeSteps(0.2f, 0, 50);
    exViewer->render();

    cout << "---- END TEST JACOBI ----" << endl;
}

void JacobiWindow::box2TCP()
{
    if (!tcp || !box || !box2)
    {
        return;
    }

    Eigen::Matrix4f m = tcp->getGlobalPose();

    box->setGlobalPose(m);

    if (elbow)
    {
        m = elbow->getGlobalPose();
        box2->setGlobalPose(m);
    }

    if (tcp2)
    {
        m = tcp2->getGlobalPose();
        box3->setGlobalPose(m);
    }

    exViewer->render();
}

void JacobiWindow::sliderPressed()
{
    cout << "GG ";
}

void JacobiWindow::loadRobot()
{
    std::cout << "JacobiWindow: Loading robot" << std::endl;
    robotSep->removeAllChildren();
    cout << "Loading Robot from " << robotFilename << endl;

    try
    {
        robot = RobotIO::loadRobot(robotFilename);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while creating robot" << endl;
        cout << e.what();
        return;
    }

    if (!robot)
    {
        cout << " ERROR while creating robot" << endl;
        return;
    }

    updateKCBox();

    if (kinChains.size() == 0)
    {
        selectKC(-1);
    }
    else
    {
        selectKC(0);
    }

    // build visualization
    collisionModel();
    exViewer->viewAll();
    exViewer->render();
}

