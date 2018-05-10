
#include "reachabilityWindow.h"
#include "VirtualRobot/EndEffector/EndEffector.h"
#include "VirtualRobot/Workspace/Reachability.h"
#include "VirtualRobot/Workspace/Manipulability.h"
#include "VirtualRobot/IK/PoseQualityExtendedManipulability.h"
#include "VirtualRobot/XML/RobotIO.h"
#include "VirtualRobot/Visualization/CoinVisualization/CoinVisualizationFactory.h"
#include <VirtualRobot/RuntimeEnvironment.h>

#include <QFileDialog>
#include <Eigen/Geometry>

#include <time.h>
#include <vector>
#include <iostream>
#include <cmath>

#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>

#include "ui_reachabilityCreate.h"

#include <sstream>
using namespace std;
using namespace VirtualRobot;

float TIMER_MS = 30.0f;


reachabilityWindow::reachabilityWindow(std::string& sRobotFile, std::string& reachFile, Eigen::Vector3f& axisTCP, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    this->axisTCP = axisTCP;
    robotFile = sRobotFile;
    useColModel = false;
    sceneSep = new SoSeparator;
    sceneSep->ref();
    robotVisuSep = new SoSeparator;
    reachabilityVisuSep = new SoSeparator;

    sceneSep->addChild(robotVisuSep);
    sceneSep->addChild(reachabilityVisuSep);

    setupUI();

    loadRobot();

    if (!reachFile.empty())
    {
        if (RuntimeEnvironment::getDataFileAbsolute(reachFile))
        {
            loadReachFile(reachFile);
        }
    }

    m_pExViewer->viewAll();
}


reachabilityWindow::~reachabilityWindow()
{
    sceneSep->unref();
}


void reachabilityWindow::setupUI()
{
    UI.setupUi(this);
    m_pExViewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    m_pExViewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    m_pExViewer->setAccumulationBuffer(true);
#ifdef WIN32
    //#ifndef _DEBUG
    m_pExViewer->setAntialiasing(true, 8);
    //#endif
#endif
    m_pExViewer->setGLRenderAction(new SoLineHighlightRenderAction);
    m_pExViewer->setTransparencyType(SoGLRenderAction::BLEND);
    m_pExViewer->setFeedbackVisibility(true);
    m_pExViewer->setSceneGraph(sceneSep);
    m_pExViewer->viewAll();

    connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonLoad, SIGNAL(clicked()), this, SLOT(selectRobot()));
    connect(UI.pushButtonLoadReachability, SIGNAL(clicked()), this, SLOT(loadReach()));
    connect(UI.pushButtonCreateReachability, SIGNAL(clicked()), this, SLOT(createReach()));
    connect(UI.pushButtonExtendReachability, SIGNAL(clicked()), this, SLOT(extendReach()));
    connect(UI.pushButtonSaveReachability, SIGNAL(clicked()), this, SLOT(saveReach()));
    connect(UI.pushButtonFillHoles, SIGNAL(clicked()), this, SLOT(fillHoles()));
    connect(UI.pushButtonBinarize, SIGNAL(clicked()), this, SLOT(binarize()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(collisionModel()));
    connect(UI.checkBoxReachabilityVisu, SIGNAL(clicked()), this, SLOT(reachVisu()));
    connect(UI.comboBoxRNS, SIGNAL(activated(int)), this, SLOT(selectRNS(int)));

    connect(UI.comboBoxJoint, SIGNAL(activated(int)), this, SLOT(selectJoint(int)));
    connect(UI.horizontalSliderPos, SIGNAL(valueChanged(int)), this, SLOT(jointValueChanged(int)));
}

QString reachabilityWindow::formatString(const char* s, float f)
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


void reachabilityWindow::resetSceneryAll()
{
    if (!robot)
    {
        return;
    }

    std::vector< RobotNodePtr > nodes;
    robot->getRobotNodes(nodes);
    std::vector<float> jv(nodes.size(), 0.0f);
    robot->setJointValues(nodes, jv);
}



void reachabilityWindow::collisionModel()
{
    if (!robot)
    {
        return;
    }

    buildVisu();
}

void reachabilityWindow::reachVisu()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    reachabilityVisuSep->removeAllChildren();

    if (UI.checkBoxReachabilityVisu->checkState() == Qt::Checked)
    {
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace, VirtualRobot::ColorMap(VirtualRobot::ColorMap::eHot), axisTCP, true);

        // different visualization
        /*
        std::vector< VirtualRobot::VisualizationFactory::Color > colors;
        VirtualRobot::VisualizationFactory::Color c1(0.5f,0.1f,0.1f,0.0f);
        VirtualRobot::VisualizationFactory::Color c2(1.0f,0.1f,0.1f,0.0f);
        colors.push_back(c1);
        colors.push_back(c2);
        VirtualRobot::ColorMap c = VirtualRobot::ColorMap::customColorMap(colors);
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace,c,true);
        */

        if (visualisationNode)
        {
            reachabilityVisuSep->addChild(visualisationNode);
        }
    }
}

void reachabilityWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void reachabilityWindow::buildVisu()
{
    if (!robot)
    {
        return;
    }

    robotVisuSep->removeAllChildren();
    useColModel = UI.checkBoxColModel->checkState() == Qt::Checked;
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    visualization = robot->getVisualization<CoinVisualization>(colModel);
    SoNode* visualisationNode = NULL;

    if (visualization)
    {
        visualisationNode = visualization->getCoinVisualization();
    }

    if (visualisationNode)
    {
        robotVisuSep->addChild(visualisationNode);
    }
}

int reachabilityWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void reachabilityWindow::quit()
{
    std::cout << "reachabilityWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void reachabilityWindow::updateRNSBox()
{
    UI.comboBoxRNS->clear();

    //UI.comboBoxRNS->addItem(QString("<All>"));
    for (unsigned int i = 0; i < robotNodeSets.size(); i++)
    {
        UI.comboBoxRNS->addItem(QString(robotNodeSets[i]->getName().c_str()));
    }

    selectRNS(0);
}

void reachabilityWindow::selectRNS(int nr)
{
    currentRobotNodeSet.reset();
    cout << "Selecting RNS nr " << nr << endl;
    std::string tcp = "<not set>";

    if (nr < 0 || nr >= (int)robotNodeSets.size())
    {
        return;
    }

    currentRobotNodeSet = robotNodeSets[nr];
    currentRobotNodes = currentRobotNodeSet->getAllRobotNodes();

    if (currentRobotNodeSet->getTCP())
    {
        tcp = currentRobotNodeSet->getTCP()->getName();
    }

    QString qTCP("TCP: ");
    qTCP += tcp.c_str();
    UI.labelTCP->setText(qTCP);
    //updateJointBox();
    //selectJoint(0);
    //displayTriangles();
}

void reachabilityWindow::updateJointBox()
{
    UI.comboBoxJoint->clear();

    for (unsigned int i = 0; i < allRobotNodes.size(); i++)
    {
        UI.comboBoxJoint->addItem(QString(allRobotNodes[i]->getName().c_str()));
    }

    selectJoint(0);
}

void reachabilityWindow::jointValueChanged(int pos)
{
    int nr = UI.comboBoxJoint->currentIndex();

    if (nr < 0 || nr >= (int)allRobotNodes.size())
    {
        return;
    }

    float fPos = allRobotNodes[nr]->getJointLimitLo() + (float)pos / 1000.0f * (allRobotNodes[nr]->getJointLimitHi() - allRobotNodes[nr]->getJointLimitLo());
    robot->setJointValue(allRobotNodes[nr], fPos);
    UI.lcdNumberJointValue->display((double)fPos);

}



void reachabilityWindow::selectJoint(int nr)
{
    currentRobotNode.reset();
    cout << "Selecting Joint nr " << nr << endl;

    if (nr < 0 || nr >= (int)allRobotNodes.size())
    {
        return;
    }

    currentRobotNode = allRobotNodes[nr];
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
}

/*
void reachabilityWindow::showCoordSystem()
{
    float size = 0.75f;
    int nr = UI.comboBoxJoint->currentIndex();
    if (nr<0 || nr>=(int)currentRobotNodes.size())
        return;

    // first check if robot node has a visualization


    currentRobotNodes[nr]->showCoordinateSystem(UI.checkBoxShowCoordSystem->checkState() == Qt::Checked, size);
    // rebuild visualization
    collisionModel();
}

*/

void reachabilityWindow::selectRobot()
{
    QString fi = QFileDialog::getOpenFileName(this, tr("Open Robot File"), QString(), tr("XML Files (*.xml)"));
    robotFile = std::string(fi.toAscii());
    loadRobot();
}

void reachabilityWindow::loadRobot()
{
    robotVisuSep->removeAllChildren();
    cout << "Loading Scene from " << robotFile << endl;

    try
    {
        robot = RobotIO::loadRobot(robotFile);
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

    // get nodes
    robot->getRobotNodes(allRobotNodes);
    std::vector < VirtualRobot::RobotNodeSetPtr > allRNS;
    robot->getRobotNodeSets(allRNS);
    robotNodeSets.clear();

    for (size_t i = 0; i < allRNS.size(); i++)
    {
        if (allRNS[i]->isKinematicChain())
        {
            VR_INFO << " RNS <" << allRNS[i]->getName() << "> is a valid kinematic chain" << endl;
            robotNodeSets.push_back(allRNS[i]);
        }
        else
        {
            VR_INFO << " RNS <" << allRNS[i]->getName() << "> is not a valid kinematic chain" << endl;
        }
    }

    updateRNSBox();
    updateJointBox();

    // build visualization
    buildVisu();
    m_pExViewer->viewAll();
}
void reachabilityWindow::extendReach()
{
    if (!robot)
    {
        return;
    }

    if (!reachSpace)
    {
        cout << " Please load/create reachability data first..." << endl;
        return;
    }

    int steps = UI.spinBoxExtend->value();
    /*#ifdef ENDLESS
        time_t time_now = time(NULL);
        struct tm * timeinfo;
        timeinfo = localtime (&time_now);
        char buffer [255];
        strftime (buffer,255,"ReachabilityData_ENDLESS_%Y_%m_%d__%H_%M_%S_",timeinfo);
        int nr = 0;
        while (true)
        {
            reachSpace->addRandomTCPPoses(steps);
            reachSpace->print();
            std::stringstream ss;
            ss << buffer << "_" << nr << ".bin";
            reachSpace->save(ss.str());
            nr++;
        }

    #endif*/
    reachSpace->addRandomTCPPoses(steps);
    reachSpace->print();
}

void reachabilityWindow::createReach()
{
    if (!robot || !currentRobotNodeSet)
    {
        return;
    }

    // setup window
    Ui::ReachabilityCreate UICreate;
    QDialog diag;
    UICreate.setupUi(&diag);
    RobotNodePtr baseNode = currentRobotNodeSet->getKinematicRoot();
    RobotNodePtr tcpNode = currentRobotNodeSet->getTCP();
    UICreate.labelRNS->setText(QString("RobotNodeSet: ") + QString(currentRobotNodeSet->getName().c_str()));
    UICreate.labelBaseNode->setText(QString("Base: ") + QString(baseNode->getName().c_str()));
    UICreate.labelTCP->setText(QString("TCP: ") + QString(tcpNode->getName().c_str()));
    reachSpace.reset(new Reachability(robot));
    float minB[6];// = {-1000.0f,-1000.0f,-1000.0f,(float)-M_PI,(float)-M_PI,(float)-M_PI};
    float maxB[6];// ={1000.0f,1000.0f,1000.0f,(float)M_PI,(float)M_PI,(float)M_PI};
    reachSpace->checkForParameters(currentRobotNodeSet, 1000, minB, maxB, baseNode, tcpNode);

    //float ex = currentRobotNodeSet->getMaximumExtension();
    UICreate.doubleSpinBoxMinX->setValue(minB[0]);
    UICreate.doubleSpinBoxMaxX->setValue(maxB[0]);
    UICreate.doubleSpinBoxMinY->setValue(minB[1]);
    UICreate.doubleSpinBoxMaxY->setValue(maxB[1]);
    UICreate.doubleSpinBoxMinZ->setValue(minB[2]);
    UICreate.doubleSpinBoxMaxZ->setValue(maxB[2]);


    std::vector < VirtualRobot::RobotNodeSetPtr > allRNS;
    robot->getRobotNodeSets(allRNS);

    for (size_t i = 0; i < allRNS.size(); i++)
    {
        UICreate.comboBoxColModelDynamic->addItem(QString(allRNS[i]->getName().c_str()));
        UICreate.comboBoxColModelStatic->addItem(QString(allRNS[i]->getName().c_str()));
    }

    UICreate.comboBoxQualityMeasure->addItem(QString("Reachability"));
    UICreate.comboBoxQualityMeasure->addItem(QString("Manipulability"));
    UICreate.comboBoxQualityMeasure->addItem(QString("Ext. Manipulability"));

    if (diag.exec())
    {


        minB[0] = UICreate.doubleSpinBoxMinX->value();
        minB[1] = UICreate.doubleSpinBoxMinY->value();
        minB[2] = UICreate.doubleSpinBoxMinZ->value();
        minB[3] = UICreate.doubleSpinBoxMinRo->value();
        minB[4] = UICreate.doubleSpinBoxMinPi->value();
        minB[5] = UICreate.doubleSpinBoxMinYa->value();
        maxB[0] = UICreate.doubleSpinBoxMaxX->value();
        maxB[1] = UICreate.doubleSpinBoxMaxY->value();
        maxB[2] = UICreate.doubleSpinBoxMaxZ->value();
        maxB[3] = UICreate.doubleSpinBoxMaxRo->value();
        maxB[4] = UICreate.doubleSpinBoxMaxPi->value();
        maxB[5] = UICreate.doubleSpinBoxMaxYa->value();

        SceneObjectSetPtr staticModel;
        SceneObjectSetPtr dynamicModel;

        if (UICreate.checkBoxColDetecion->isChecked())
        {
            std::string staticM = std::string(UICreate.comboBoxColModelStatic->currentText().toAscii());
            std::string dynM = std::string(UICreate.comboBoxColModelDynamic->currentText().toAscii());
            staticModel = robot->getRobotNodeSet(staticM);
            dynamicModel = robot->getRobotNodeSet(dynM);
        }

        float discrTr = UICreate.doubleSpinBoxDiscrTrans->value();
        float discrRo = UICreate.doubleSpinBoxDiscrRot->value();

        std::string measure = std::string(UICreate.comboBoxQualityMeasure->currentText().toAscii());
        if (measure!="Reachability")
        {
            reachSpace.reset(new Manipulability(robot));
            ManipulabilityPtr manipSpace = boost::dynamic_pointer_cast<Manipulability>(reachSpace);
            manipSpace->setMaxManipulability(UICreate.doubleSpinBoxMaxManip->value());
        }

        reachSpace->initialize(currentRobotNodeSet, discrTr, discrRo, minB, maxB, staticModel, dynamicModel, baseNode, tcpNode); //200.0f,0.4f,minB,maxB,staticModel,dynamicModel,baseNode);
        if (measure == "Ext. Manipulability")
        {
            ManipulabilityPtr man = boost::dynamic_pointer_cast<Manipulability>(reachSpace);
            PoseQualityExtendedManipulabilityPtr manMeasure(new PoseQualityExtendedManipulability(currentRobotNodeSet));
            man->setManipulabilityMeasure(manMeasure);
        }
        reachSpace->print();

        reachSpace->addCurrentTCPPose();
        reachSpace->print();
    }
}

void reachabilityWindow::fillHoles()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    cout << "filling holes of reachability space" << endl;
    int res = reachSpace->fillHoles();
    cout << "Filled " << res << " voxels" << endl;
    reachSpace->print();
}

void reachabilityWindow::binarize()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    cout << "Binarizing reachability space" << endl;
    reachSpace->binarize();
    reachSpace->print();
}

void reachabilityWindow::saveReach()
{
    if (!robot || !reachSpace)
    {
        return;
    }

    QString fi = QFileDialog::getSaveFileName(this, tr("Save Reachability to File"), QString(), tr("bin Files (*.bin);;all Files (*.*)"));

    if (fi.isEmpty())
    {
        return;
    }

    reachFile = std::string(fi.toAscii());
    reachSpace->save(reachFile);

}
void reachabilityWindow::loadReachFile(std::string filename)
{
    if (!robot)
    {
        return;
    }

    reachFile = filename;
    bool loadOK = true;

    // try manipulability file
    try
    {
        reachSpace.reset(new Manipulability(robot));
        reachSpace->load(reachFile);
    } catch (...)
    {
        loadOK = false;
    }

    if (!loadOK)
    {
        // try reachability file

        loadOK = true;
        try {
            reachSpace.reset(new Reachability(robot));
            reachSpace->load(reachFile);
        } catch (...)
        {
            loadOK = false;
        }
    }

    if (!loadOK)
    {
        VR_ERROR << "Could not load reach/manip file" << endl;
        reachSpace.reset();
        return;
    }

    reachSpace->print();

    if (reachSpace->getNodeSet())
    {
        cout << "Using RNS: " << reachSpace->getNodeSet()->getName() << endl;

        for (size_t i = 0; i < robotNodeSets.size(); i++)
        {
            cout << "checking " << robotNodeSets[i]->getName() << endl;

            if (robotNodeSets[i] == reachSpace->getNodeSet())
            {
                cout << "Found RNS.." << endl;
                UI.comboBoxRNS->setCurrentIndex(i);
                selectRNS(i);
            }
        }
    }
}

void reachabilityWindow::loadReach()
{
    if (!robot)
    {
        return;
    }

    QString fi = QFileDialog::getOpenFileName(this, tr("Open Reachability File"), QString(), tr("bin Files (*.bin);;all Files (*.*)"));

    if (fi.isEmpty())
    {
        return;
    }

    reachFile = std::string(fi.toAscii());
    loadReachFile(reachFile);
}
