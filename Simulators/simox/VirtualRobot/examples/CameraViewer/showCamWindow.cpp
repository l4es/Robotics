
#include "showCamWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/Import/RobotImporterFactory.h>

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/nodes/SoUnits.h>

#include <sstream>

using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;

showCamWindow::showCamWindow(std::string& sRobotFilename, std::string& cam1Name, std::string& cam2Name, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;
    //this->setCaption(QString("ShowRobot - KIT - Humanoids Group"));
    //resize(1100, 768);
    cam1Renderer = NULL;
    cam2Renderer = NULL;
    cam1Buffer = NULL;
    cam2Buffer = NULL;

    useColModel = false;
    VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(sRobotFilename);
    m_sRobotFilename = sRobotFilename;
    this->cam1Name = cam1Name;
    this->cam2Name = cam2Name;
    sceneSep = new SoSeparator;
    sceneSep->ref();
    /*SoUnits *u = new SoUnits;
    u->units = SoUnits::MILLIMETERS;
    sceneSep->addChild(u);*/
    robotSep = new SoSeparator;
    extraSep = new SoSeparator;
    sceneSep->addChild(extraSep);
    visuObject = VirtualRobot::Obstacle::createSphere(400.0f);
    Eigen::Matrix4f m;
    m.setIdentity();
    m(1,3) = 1500.0f;
    m(2,3) = 1500.0f;
    visuObject->setGlobalPose(m);
    SoNode* objectSep = VirtualRobot::CoinVisualizationFactory::getCoinVisualization(visuObject,VirtualRobot::SceneObject::Full);
    extraSep->addChild(objectSep); 

    /*SoShapeHints * shapeHints = new SoShapeHints;
    shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
    shapeHints->shapeType = SoShapeHints::UNKNOWN_SHAPE_TYPE;
    sceneSep->addChild(shapeHints);*/
    /*SoLightModel * lightModel = new SoLightModel;
    lightModel->model = SoLightModel::BASE_COLOR;
    sceneSep->addChild(lightModel);*/

    sceneSep->addChild(robotSep);

    setupUI();

    loadRobot();

    viewer->viewAll();
}


showCamWindow::~showCamWindow()
{
    robot.reset();
    sceneSep->unref();

    UI.cam1->setScene(NULL);
    UI.cam2->setScene(NULL);

    delete [] cam1Buffer;
    delete [] cam2Buffer;
}



void showCamWindow::setupUI()
{
    UI.setupUi(this);
    //centralWidget()->setLayout(UI.gridLayoutViewer);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
#ifdef WIN32
    //#ifndef _DEBUG
    viewer->setAntialiasing(true, 4);
    //#endif
#endif
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));

    connect(UI.comboBoxRobotNodeSet, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));
    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));

}

QString showCamWindow::formatString(const char* s, float f)
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


void showCamWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector<float> jv(allRobotNodes.size(), 0.0f);
    robot->setJointValues(allRobotNodes, jv);

    selectJoint(UI.comboBoxJoint->currentIndex());
}



void showCamWindow::rebuildVisualization()
{
    if (!robot)
    {
        return;
    }

    robotSep->removeAllChildren();
    //setRobotModelShape(UI.checkBoxColModel->state() == QCheckBox::On);
    useColModel = false;
    //bool sensors = UI.checkBoxRobotSensors->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = useColModel ? SceneObject::Collision : SceneObject::Full;

    visualization = robot->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotSep->addChild(visualisationNode);
    }

    selectJoint(UI.comboBoxJoint->currentIndex());
}



void showCamWindow::showRobot()
{
    //m_pGraspScenery->showRobot(m_pShowRobot->state() == QCheckBox::On);
}

void showCamWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}




int showCamWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void showCamWindow::quit()
{
    std::cout << "CShowRobotWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void showCamWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (unsigned int i = 0; i < currentRobotNodes.size(); i++)
    {
        UI.comboBoxJoint->addItem(QString(currentRobotNodes[i]->getName().c_str()));
    }
}

void showCamWindow::updateRNSBox()
{
    UI.comboBoxRobotNodeSet->clear();
    UI.comboBoxRobotNodeSet->addItem(QString("<All>"));

    for (unsigned int i = 0; i < robotNodeSets.size(); i++)
    {
        UI.comboBoxRobotNodeSet->addItem(QString(robotNodeSets[i]->getName().c_str()));
    }
}

void showCamWindow::selectRNS(int nr)
{
    currentRobotNodeSet.reset();
    cout << "Selecting RNS nr " << nr << endl;

    if (nr <= 0)
    {
        // all joints
        currentRobotNodes = allRobotNodes;
    }
    else
    {
        nr--;

        if (nr >= (int)robotNodeSets.size())
        {
            return;
        }

        currentRobotNodeSet = robotNodeSets[nr];
        currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();
        /*cout << "HIGHLIGHTING rns " << currentRobotNodeSet->getName() << endl;
        if (visualization)
        {

            robot->highlight(visualization,false);
            currentRobotNodeSet->highlight(visualization,true);
        }*/

    }

    updateJointBox();
    selectJoint(0);
}

void showCamWindow::selectJoint(int nr)
{
    if (currentRobotNode)
    {
        currentRobotNode->showBoundingBox(false);
    }

    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    currentRobotNode = currentRobotNodes[nr];
    currentRobotNode->showBoundingBox(true, true);
    currentRobotNode->print();
    float mi = currentRobotNode->getJointLimitLo();
    float ma = currentRobotNode->getJointLimitHi();
    QString qMin = QString::number(mi);
    QString qMax = QString::number(ma);
    UI.labelMinPos->setText(qMin);
    UI.labelMaxPos->setText(qMax);
    float j = currentRobotNode->getJointValue();
    UI.lcdNumberJointValue->display((double)j);

    if (fabs(ma - mi) > 0 && (currentRobotNode->isTranslationalJoint() || currentRobotNode->isRotationalJoint()))
    {
        UI.horizontalSliderPos->setEnabled(true);
        int pos = (int)((j - mi) / (ma - mi) * 1000.0f);
        UI.horizontalSliderPos->setValue(pos);
    }
    else
    {
        UI.horizontalSliderPos->setValue(500);
        UI.horizontalSliderPos->setEnabled(false);
    }

    cout << "HIGHLIGHTING node " << currentRobotNodes[nr]->getName() << endl;

    if (visualization)
    {
        robot->highlight(visualization, false);
        currentRobotNode->highlight(visualization, true);
    }

}

void showCamWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)currentRobotNodes.size())
    {
        return;
    }

    float fPos = currentRobotNodes[nr]->getJointLimitLo() + (float)pos / 1000.0f * (currentRobotNodes[nr]->getJointLimitHi() - currentRobotNodes[nr]->getJointLimitLo());
    robot->setJointValue(currentRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display((double)fPos);

    renderCam();
}



void showCamWindow::selectRobot()
{
    string supportedExtensions = RobotImporterFactory::getAllExtensions();
    string supported = "Supported Formats, " + supportedExtensions + " (" + supportedExtensions + ")";
    string filter = supported + ";;" + RobotImporterFactory::getAllFileFilters();
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr(filter.c_str()));
    std::string s = m_sRobotFilename = std::string(fi.toAscii());

    if (!s.empty())
    {
        m_sRobotFilename = s;
        loadRobot();
    }
}

void showCamWindow::loadRobot()
{
    robotSep->removeAllChildren();
    cout << "Loading Robot from " << m_sRobotFilename << endl;
    currentRobotNode.reset();
    currentRobotNodes.clear();
    currentRobotNodeSet.reset();
    robot.reset();

    try
    {
        QFileInfo fileInfo(m_sRobotFilename.c_str());
        std::string suffix(fileInfo.suffix().toAscii());
        RobotImporterFactoryPtr importer = RobotImporterFactory::fromFileExtension(suffix, NULL);

        if (!importer)
        {
            cout << " ERROR while grabbing importer" << endl;
            return;
        }

        robot = importer->loadFromFile(m_sRobotFilename, RobotIO::eFull);


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
    updateCameras();
    updatRobotInfo();
}

void showCamWindow::updateCameras()
{
    cam1.reset();
    cam2.reset();
    cam1Renderer = NULL;
    cam2Renderer = NULL;
    delete []cam1Buffer;
    cam1Buffer = NULL;
    delete []cam2Buffer;
    cam2Buffer = NULL;
    if (!robot)
        return;
    if (robot->hasRobotNode(cam1Name))
        cam1 = robot->getRobotNode(cam1Name);
    if (robot->hasRobotNode(cam2Name))
        cam2 = robot->getRobotNode(cam2Name);

    if (cam1)
    {
        cam1Renderer = CoinVisualizationFactory::createOffscreenRenderer(UI.cam1->size().width(), UI.cam1->size().height());
        cam1Buffer = new unsigned char [UI.cam1->size().width() * UI.cam1->size().height() * 3];
    }
    if (cam2)
    {
        cam2Renderer = CoinVisualizationFactory::createOffscreenRenderer(UI.cam2->size().width(), UI.cam2->size().height());
        cam2Buffer = new unsigned char [UI.cam2->size().width() * UI.cam2->size().height() * 3];
    }

    renderCam();
}

void showCamWindow::renderCam()
{
    if (cam1 && cam1Renderer)
    {
        CoinVisualizationFactory::renderOffscreen(cam1Renderer, cam1, sceneSep, &cam1Buffer);
        QImage img1(cam1Buffer, UI.cam1->size().width(), UI.cam1->size().height(), QImage::Format_RGB888);
        //UI.cam1->setPixmap(QPixmap::fromImage(img1));

        QGraphicsScene *scene = new QGraphicsScene();
        //scene->addPixmap(QPixmap::fromImage(qimg2.mirrored(true,false))); // we need to mirror the image, since different coord systems are assumed
        scene->addPixmap(QPixmap::fromImage(img1.mirrored(false,true))); // we need to mirror the image as the output from the renderer is of "left-bottom" type
        QGraphicsScene *oldScene = UI.cam1->scene();
        UI.cam1->setScene(scene);
        delete oldScene; 

    }
    if (cam2 && cam2Renderer)
    {
        CoinVisualizationFactory::renderOffscreen(cam2Renderer, cam2, sceneSep, &cam2Buffer);
        QImage img2(cam2Buffer, UI.cam2->size().width(), UI.cam2->size().height(), QImage::Format_RGB888);
        //UI.cam2->setPixmap(QPixmap::fromImage(img2));

        QGraphicsScene *scene = new QGraphicsScene();
        //scene->addPixmap(QPixmap::fromImage(qimg2.mirrored(true,false))); // we need to mirror the image, since different coord systems are assumed
        scene->addPixmap(QPixmap::fromImage(img2.mirrored(false,true))); // we need to mirror the image as the output from the renderer is of "left-bottom" type
        QGraphicsScene *oldScene = UI.cam2->scene();
        UI.cam2->setScene(scene);
        delete oldScene; 
    }
}

void showCamWindow::updatRobotInfo()
{
    if (!robot)
    {
        return;
    }
    // get nodes
    robot->getRobotNodes(allRobotNodes);
    robot->getRobotNodeSets(robotNodeSets);

    updateRNSBox();
    selectRNS(0);

    if (allRobotNodes.size() == 0)
    {
        selectJoint(-1);
    }
    else
    {
        selectJoint(0);
    }

    // build visualization
    rebuildVisualization();

    viewer->viewAll();
}

