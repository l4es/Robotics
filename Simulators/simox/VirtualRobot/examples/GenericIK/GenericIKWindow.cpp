
#include "GenericIKWindow.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h"
#include "VirtualRobot/EndEffector/EndEffector.h"

#include <time.h>
#include <vector>
#include <iostream>
#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

GenericIKWindow::GenericIKWindow(std::string& sRobotFilename, Qt::WFlags flags)
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
    box->showCoordinateSystem(true);
    boxSep = new SoSeparator();
    boxSep->addChild(CoinVisualization(box->getVisualization()).getCoinVisualization());
    sceneSep->addChild(boxSep);
    box2TCP();

    exViewer->viewAll();
}


GenericIKWindow::~GenericIKWindow()
{
    sceneSep->unref();
}

void GenericIKWindow::setupUI()
{
    UI.setupUi(this);
    exViewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    exViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    exViewer->setAccumulationBuffer(true);
    exViewer->setAntialiasing(true, 4);

    exViewer->setTransparencyType(SoGLRenderAction::BLEND);
    exViewer->setFeedbackVisibility(true);
    exViewer->setSceneGraph(sceneSep);
    exViewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(loadRobot()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.comboBoxKC, SIGNAL(activated(int)), this, SLOT(selectKC(int)));

    connect(UI.pushButtonBox2TCP, SIGNAL(clicked()), this, SLOT(box2TCP()));
    connect(UI.pushButtonSolve, SIGNAL(clicked()), this, SLOT(solve()));


    connect(UI.horizontalSliderX, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderY, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderZ, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderA, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderB, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));
    connect(UI.horizontalSliderG, SIGNAL(sliderReleased()), this, SLOT(sliderReleased()));

    // a method, that is called by a timer, is allowed to update the IV models without disturbing the render loop
    SoTimerSensor* timer = new SoTimerSensor((SoSensorCB*)(&updateCB), this);
    SbTime interval(1.0 / 30);
    timer->setInterval(interval);
    timer->schedule();
}

void GenericIKWindow::updateCB(void* data, SoSensor* sensor)
{
    if (!data)
    {
        return;
    }

    GenericIKWindow* w = (GenericIKWindow*)(data);

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
}

void GenericIKWindow::updatBoxPos(float x, float y, float z, float a, float b, float g)
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

QString GenericIKWindow::formatString(const char* s, float f)
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


void GenericIKWindow::resetSceneryAll()
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



void GenericIKWindow::collisionModel()
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


void GenericIKWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


int GenericIKWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void GenericIKWindow::quit()
{
    std::cout << "GenericIKWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void GenericIKWindow::updateKCBox()
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

void GenericIKWindow::selectKC(int nr)
{
    cout << "Selecting kinematic chain nr " << nr << endl;

    if (nr < 0 || nr >= (int)kinChains.size())
    {
        return;
    }

    if (tcp)
    {
        tcp->showCoordinateSystem(false);
    }

    /*std::vector<RobotNodePtr> nodes0;
    if (kc)
        nodes0 = kc->getAllRobotNodes();
    for (size_t i=0;i<nodes0.size();i++)
    {
        nodes0[i]->showBoundingBox(false,true);
    }*/
    kc = kinChains[nr];
    kc->print();
    QString nameQ = "TCP: ";
    tcp = kc->getTCP();

    if (tcp)
    {
        QString n(tcp->getName().c_str());
        nameQ += n;
        tcp->showCoordinateSystem(true);
    }

    unsigned int d = kc->getSize();
    QString qd = "Nr of joints: ";
    qd += QString::number(d);

    UI.label_TCP->setText(nameQ);
    UI.label_NrJoints->setText(qd);
    /*std::vector<RobotNodePtr> nodes = kc->getAllRobotNodes();
    for (size_t i=0;i<nodes.size();i++)
    {
        nodes[i]->showBoundingBox(true,true);
    }*/
    box2TCP();

    if (kc->getNode(kc->getSize() - 1)->isTranslationalJoint())
    {
        ikGazeSolver.reset(new GazeIK(kc, boost::dynamic_pointer_cast<RobotNodePrismatic>(kc->getNode(kc->getSize() - 1))));
    }
    else
    {
        ikGazeSolver.reset();
    }

    ikSolver.reset(new GenericIKSolver(kc, JacobiProvider::eSVDDamped));
    //ikSolver->getDifferentialIK()->setVerbose(true);
    /*Eigen::VectorXf js(d);
    js.setConstant(1.0f);
    js(js.rows() - 1) = 10.0f;
    ikSolver->getDifferentialIK()->setJointWeights(js);*/
    ikSolver->getDifferentialIK()->setMaxPositionStep(20.0f);
    ikSolver->setupJacobian(0.3f, 100);

    // since it may be that a tcp coord system was created in this method we must re-build the visualization in order to show it
    collisionModel();
}

void GenericIKWindow::sliderReleased()
{
    UI.horizontalSliderX->setSliderPosition(0);
    UI.horizontalSliderY->setSliderPosition(0);
    UI.horizontalSliderZ->setSliderPosition(0);
    UI.horizontalSliderA->setSliderPosition(0);
    UI.horizontalSliderB->setSliderPosition(0);
    UI.horizontalSliderG->setSliderPosition(0);
    exViewer->render();
}


void GenericIKWindow::solve()
{
    if (!kc || !ikSolver || !tcp)
    {
        return;
    }

    cout << "---- Solve IK ----" << endl;

    IKSolver::CartesianSelection s = IKSolver::All;

    if (UI.radioButton_Pos->isChecked())
    {
        s = IKSolver::Position;
    }

    //if (UI.radioButton_Ori->isChecked())
    //  s = IKSolver::Orientation;
    //ikSolver->setVerbose(true);
    Eigen::Matrix4f targetPose = box->getGlobalPose();

    /*
    if (kc && kc->getNode(kc->getSize() - 1)->isTranslationalJoint() && kc->getNode(kc->getSize() - 1)->getParent())
    {
        // setup gaze IK
        float v = (kc->getNode(kc->getSize() - 1)->getParent()->getGlobalPose().block(0, 3, 3, 1) - targetPose.block(0, 3, 3, 1)).norm();
        cout << "Setting initial value of translation joint to :" << v << endl;
        ikSolver->setupTranslationalJoint(kc->getNode(kc->getSize() - 1), v);
        kc->getNode(kc->getSize() - 1)->setJointValue(v);
    }*/
    clock_t startT = clock();

    if (ikGazeSolver)
    {
        ikGazeSolver->solve(targetPose.block(0, 3, 3, 1));
    }
    else
    {
        ikSolver->solve(targetPose, s, 50);
    }

    clock_t endT = clock();

    Eigen::Matrix4f actPose = tcp->getGlobalPose();
    float errorPos = (actPose.block(0, 3, 3, 1) - targetPose.block(0, 3, 3, 1)).norm();
    MathTools::Quaternion q1 = MathTools::eigen4f2quat(actPose);
    MathTools::Quaternion q2 = MathTools::eigen4f2quat(targetPose);
    MathTools::Quaternion d = getDelta(q1, q2);
    float errorOri = fabs(180.0f - (d.w + 1.0f) * 90.0f);

    float diffClock = (float)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0f);
    QString qd = "Time: ";
    qd += QString::number(diffClock, 'f', 2);
    qd += " ms";
    UI.labelTime->setText(qd);
    QString qd2 = "Error Pos: : ";
    qd2 += QString::number(errorPos, 'f', 2);
    qd2 += " mm";
    UI.labelPos->setText(qd2);
    QString qd3 = "Error Ori: : ";
    qd3 += QString::number(errorOri, 'f', 2);
    qd3 += " deg";
    UI.labelOri->setText(qd3);

    cout << "Joint values:" << endl;
    std::vector<RobotNodePtr> nodes = kc->getAllRobotNodes();

    for (size_t i = 0; i < nodes.size(); i++)
    {
        cout << nodes[i]->getJointValue() << endl;
    }

    /*
    DifferentialIKPtr j(new DifferentialIK(kc));
    j->setGoal(targetPose,RobotNodePtr(),IKSolver::All);
    j->computeSteps(0.2f,0,50);
    */
    exViewer->render();

    cout << "---- END Solve IK ----" << endl;
}

void GenericIKWindow::box2TCP()
{
    if (!tcp || !box)
    {
        return;
    }

    Eigen::Matrix4f m = tcp->getGlobalPose();

    box->setGlobalPose(m);
    exViewer->render();
}

void GenericIKWindow::sliderPressed()
{
}

void GenericIKWindow::loadRobot()
{
    std::cout << "GenericIKWindow: Loading robot" << std::endl;
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

