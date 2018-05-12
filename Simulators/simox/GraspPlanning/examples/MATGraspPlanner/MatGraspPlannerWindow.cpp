/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    GraspStudio
* @author     Markus Przybylski
* @copyright  2013 H2T,KIT
*             GNU Lesser General Public License
*
*/

#include "MatGraspPlannerWindow.h"
#include "GraspPlanning/Visualization/CoinVisualization/CoinConvexHullVisualization.h"
#include "GraspPlanning/ContactConeGenerator.h"
#include "GraspPlanning/GraspPlanner/MATPlanner/CoinVisualization/DrawHelpers.h"
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
#include "VirtualRobot/Visualization/TriMeshModel.h"
#include <QFileDialog>
#include <Eigen/Geometry>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/FileIO.h>

#include <stdlib.h>

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
#include <Inventor/nodes/SoScale.h>

#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoSphere.h>
#include <sstream>
using namespace std;
using namespace VirtualRobot;
using namespace GraspStudio;

float TIMER_MS = 30.0f;

MatGraspPlannerWindow::MatGraspPlannerWindow(std::string& robFile, std::string& eefName, std::string& preshape, std::string& objFile, Qt::WFlags flags)
    : QMainWindow(NULL)
{
    VR_INFO << " start " << endl;

    // init the random number generator
    srand(time(NULL));

    this->robotFile = robFile;
    this->objectFile = objFile;
    this->eefName = eefName;
    this->preshape = preshape;
    //eefVisu = new SoSeparator;
    //eefVisu->ref();
    sceneSep = new SoSeparator;
    sceneSep->ref();
    eefClonedSep = new SoSeparator;
    objectSep = new SoSeparator;
    frictionConeSep = new SoSeparator;
    graspsSep = new SoSeparator;
    graspsSep->ref();

    //MP 2013-06-14
    drawStuffSep = new SoSeparator;

    drawStuffGuiSep = new SoSeparator;
    drawStuffGuiSep->ref();
    medialAxisPointCloudSep = new SoSeparator;
    medialAxisPointCloudSep->ref();

    medialSpheresSep = new SoSeparator;
    medialSpheresSep->ref();

    medialAxisPointCloudFilteredSep = new SoSeparator;
    medialAxisPointCloudFilteredSep->ref();
    medialSpheresFilteredSep = new SoSeparator;
    medialSpheresFilteredSep->ref();

    neighborhoodSep = new SoSeparator;
    neighborhoodSep->ref();
    searchRadiusSep = new SoSeparator;
    searchRadiusSep->ref();
    candidateGraspsSep = new SoSeparator;
    candidateGraspsSep->ref();


#if 0
    SoSeparator* s = CoinVisualizationFactory::CreateCoordSystemVisualization();
    sceneSep->addChild(s);
#endif
    sceneSep->addChild(eefClonedSep);
    sceneSep->addChild(objectSep);
    sceneSep->addChild(frictionConeSep);
    //sceneSep->addChild(eefVisu);
    //sceneSep->addChild(graspsSep);

    //MP 2013-06-14
    sceneSep->addChild(drawStuffSep);

    sceneSep->addChild(drawStuffGuiSep);
    drawStuffGuiSep->addChild(medialAxisPointCloudSep);
    drawStuffGuiSep->addChild(medialSpheresSep);
    drawStuffGuiSep->addChild(neighborhoodSep);
    drawStuffGuiSep->addChild(searchRadiusSep);
    drawStuffGuiSep->addChild(candidateGraspsSep);

    drawStuffGuiSep->addChild(medialAxisPointCloudFilteredSep);
    drawStuffGuiSep->addChild(medialSpheresFilteredSep);

    setupUI();

    loadRobot();
    loadObject();

    buildVisu();



    viewer->viewAll();

    candidateTestCounter = 0;

    //initializing the configuration
    gpConfig = GraspPlannerConfigurationPtr(new GraspPlannerConfiguration);

}


MatGraspPlannerWindow::~MatGraspPlannerWindow()
{
    sceneSep->unref();
    graspsSep->unref();
    //eefVisu->unref();
}



void MatGraspPlannerWindow::setupUI()
{
    UI.setupUi(this);
    viewer = new SoQtExaminerViewer(UI.frameViewer, "", TRUE, SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
#ifdef WIN32
    viewer->setAntialiasing(true, 8);
#endif
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::SORTED_OBJECT_BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();

    //  connect(UI.pushButtonReset, SIGNAL(clicked()), this, SLOT(resetSceneryAll()));
    connect(UI.pushButtonSelectObject, SIGNAL(clicked()), this, SLOT(selectObject()));
    //  connect(UI.pushButtonPlan, SIGNAL(clicked()), this, SLOT(plan()));
    connect(UI.pushButtonSave, SIGNAL(clicked()), this, SLOT(save()));
    //  connect(UI.pushButtonOpen, SIGNAL(clicked()), this, SLOT(openEEF()));
    //  connect(UI.pushButtonClose, SIGNAL(clicked()), this, SLOT(closeEEF()));
    connect(UI.pushButtonShowGrasp, SIGNAL(clicked()), this, SLOT(showNextGrasp()));
    connect(UI.pushButtonTestAllCandidates, SIGNAL(clicked()), this, SLOT(testAllCandidates()));
    connect(UI.pushButtonTestStuff, SIGNAL(clicked()), this, SLOT(testStuff()));

    connect(UI.checkBoxColModel, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCones, SIGNAL(clicked()), this, SLOT(frictionConeVisu()));
    connect(UI.checkBoxGrasps, SIGNAL(clicked()), this, SLOT(showGrasps()));
    connect(UI.checkBoxHand, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxObject, SIGNAL(clicked()), this, SLOT(colModel()));

    // MP 2013-06-14 first tests
    connect(UI.pushButtonInit, SIGNAL(clicked()), this, SLOT(initPlanner()));
    //connect(UI.pushButtonDrawStuff, SIGNAL(clicked()), this, SLOT(drawStuff()));
    connect(UI.pushButtonRemoveStuff, SIGNAL(clicked()), this, SLOT(removeStuff()));

    //MP 2013-09-25
    connect(UI.checkBoxMedialAxisComplete, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxMedialSpheres, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxMedialAxisFiltered, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxMedialSpheresFiltered, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxLocalNeighborhoods, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxSearchRadius, SIGNAL(clicked()), this, SLOT(colModel()));
    connect(UI.checkBoxCandidateGrasps, SIGNAL(clicked()), this, SLOT(colModel()));
}


void MatGraspPlannerWindow::resetSceneryAll()
{
    grasps->removeAllGrasps();
    graspsSep->removeAllChildren();

    //if (rns)
    //  rns->setJointValues(startConfig);
}


void MatGraspPlannerWindow::closeEvent(QCloseEvent* event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void MatGraspPlannerWindow::buildVisu()
{

    eefClonedSep->removeAllChildren();
    SceneObject::VisualizationType colModel = (UI.checkBoxColModel->isChecked()) ? SceneObject::Collision : SceneObject::Full;

    if (eefCloned && UI.checkBoxHand->isChecked())
    {
        visualizationEEFCloned = eefCloned->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationEEFCloned->getCoinVisualization();

        if (visualisationNode)
        {
            eefClonedSep->addChild(visualisationNode);
            visualizationEEFCloned->highlight(UI.checkBoxHighlight->isChecked());
        }
    }

    /*
    if (robot)
    {
        visualizationRobot = robot->getVisualization<CoinVisualization>(colModel);
        SoNode* visualisationNode = visualizationRobot->getCoinVisualization();
        if (visualisationNode)
        {
            robotSep->addChild(visualisationNode);
            //visualizationRobot->highlight(true);
        }
    }
    */
    objectSep->removeAllChildren();

    if (object && UI.checkBoxObject->isChecked())
    {
        SceneObject::VisualizationType colModel2 = (UI.checkBoxColModel->isChecked()) ? SceneObject::CollisionData : SceneObject::Full;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(object, colModel2);

        if (visualisationNode)
        {
            objectSep->addChild(visualisationNode);
        }

        /*SoNode *s = CoinVisualizationFactory::getCoinVisualization(object->getCollisionModel()->getTriMeshModel(),true);
        if (s)
            objectSep->addChild(s);   */
    }

    frictionConeSep->removeAllChildren();
    bool fc = (UI.checkBoxCones->isChecked());

    if (fc && contacts.size() > 0 && qualityMeasure)
    {
        ContactConeGeneratorPtr cg = qualityMeasure->getConeGenerator();
        float radius = cg->getConeRadius();
        float height = cg->getConeHeight();
        float scaling = 30.0f;
        SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(contacts, height * scaling, radius * scaling, true);

        if (visualisationNode)
        {
            frictionConeSep->addChild(visualisationNode);
        }

        // add approach dir visu
        for (size_t i = 0; i < contacts.size(); i++)
        {
            SoSeparator* s = new SoSeparator;
            Eigen::Matrix4f ma;
            ma.setIdentity();
            ma.block(0, 3, 3, 1) = contacts[i].contactPointFingerGlobal;
            SoMatrixTransform* m = CoinVisualizationFactory::getMatrixTransform(ma);
            s->addChild(m);
            s->addChild(CoinVisualizationFactory::CreateArrow(contacts[i].approachDirectionGlobal, 10.0f, 1.0f));
            frictionConeSep->addChild(s);
        }
    }

    if (UI.checkBoxGrasps->isChecked() && sceneSep->findChild(graspsSep) < 0)
    {
        sceneSep->addChild(graspsSep);
    }

    if (!UI.checkBoxGrasps->isChecked() && sceneSep->findChild(graspsSep) >= 0)
    {
        sceneSep->removeChild(graspsSep);
    }

    //MP 2013-09-25 draw more stuff using checkboxes
    //drawStuffGuiSep->removeAllChildren();

    // ---- medial axis
    if (UI.checkBoxMedialAxisComplete->isChecked()
        && drawStuffGuiSep->findChild(medialAxisPointCloudSep) < 0)
    {
        drawStuffGuiSep->addChild(medialAxisPointCloudSep);
    }

    if (!UI.checkBoxMedialAxisComplete->isChecked()
        && drawStuffGuiSep->findChild(medialAxisPointCloudSep) >= 0)
    {
        drawStuffGuiSep->removeChild(medialAxisPointCloudSep);
    }

    if (UI.checkBoxMedialAxisFiltered->isChecked()
        && drawStuffGuiSep->findChild(medialAxisPointCloudFilteredSep) < 0)
    {
        drawStuffGuiSep->addChild(medialAxisPointCloudFilteredSep);
    }

    if (!UI.checkBoxMedialAxisFiltered->isChecked()
        && drawStuffGuiSep->findChild(medialAxisPointCloudFilteredSep) >= 0)
    {
        drawStuffGuiSep->removeChild(medialAxisPointCloudFilteredSep);
    }

    // ---- medial spheres
    if (UI.checkBoxMedialSpheres->isChecked()
        && drawStuffGuiSep->findChild(medialSpheresSep) < 0)
    {
        drawStuffGuiSep->addChild(medialSpheresSep);
    }

    if (!UI.checkBoxMedialSpheres->isChecked()
        && drawStuffGuiSep->findChild(medialSpheresSep) >= 0)
    {
        drawStuffGuiSep->removeChild(medialSpheresSep);
    }

    if (UI.checkBoxMedialSpheresFiltered->isChecked()
        && drawStuffGuiSep->findChild(medialSpheresFilteredSep) < 0)
    {
        drawStuffGuiSep->addChild(medialSpheresFilteredSep);
    }

    if (!UI.checkBoxMedialSpheresFiltered->isChecked()
        && drawStuffGuiSep->findChild(medialSpheresFilteredSep) >= 0)
    {
        drawStuffGuiSep->removeChild(medialSpheresFilteredSep);
    }

    // ---- local neighborhoods
    if (UI.checkBoxLocalNeighborhoods->isChecked()
        && drawStuffGuiSep->findChild(neighborhoodSep) < 0)
    {
        drawStuffGuiSep->addChild(neighborhoodSep);
    }

    if (!UI.checkBoxLocalNeighborhoods->isChecked()
        && drawStuffGuiSep->findChild(neighborhoodSep) >= 0)
    {
        drawStuffGuiSep->removeChild(neighborhoodSep);
    }

    // ---- search radius
    if (UI.checkBoxSearchRadius->isChecked()
        && drawStuffGuiSep->findChild(searchRadiusSep) < 0)
    {
        drawStuffGuiSep->addChild(searchRadiusSep);
    }

    if (!UI.checkBoxSearchRadius->isChecked()
        && drawStuffGuiSep->findChild(searchRadiusSep) >= 0)
    {
        drawStuffGuiSep->removeChild(searchRadiusSep);
    }

    // ---- candidate grasps
    if (UI.checkBoxCandidateGrasps->isChecked()
        && drawStuffGuiSep->findChild(candidateGraspsSep) < 0)
    {
        drawStuffGuiSep->addChild(candidateGraspsSep);
    }

    if (!UI.checkBoxCandidateGrasps->isChecked()
        && drawStuffGuiSep->findChild(candidateGraspsSep) >= 0)
    {
        drawStuffGuiSep->removeChild(candidateGraspsSep);
    }




    viewer->scheduleRedraw();
}

int MatGraspPlannerWindow::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}


void MatGraspPlannerWindow::quit()
{
    std::cout << "MatGraspPlannerWindow: Closing" << std::endl;
    this->close();
    SoQt::exitMainLoop();
}

void MatGraspPlannerWindow::loadObject()
{
    if (!eef)
    {
        VR_ERROR << "No end effector loaded..." << endl;
        return;
    }

    if (!object)
    {
        object = Obstacle::createBox(100.0f, 100.0f, 40.0f);
    }

    object = MeshConverter::refineObjectSurface(object, 4.0f);


    qualityMeasure.reset(new GraspStudio::GraspQualityMeasureWrenchSpace(object));
    //qualityMeasure->setVerbose(true);
    qualityMeasure->calculateObjectProperties();
}

void MatGraspPlannerWindow::loadObjectFromFile(string objectFilename)
{

    object = ObjectIO::loadManipulationObject(objectFilename);

    if (UI.checkBoxRefineMesh->isChecked())
    {
        cout << "Refining surface mesh..." << endl;
        object = MeshConverter::refineObjectSurface(object, 4.0f);
        cout << "Refining surface mesh: Done." << endl;
    }

}

void MatGraspPlannerWindow::loadRobot()
{
    robot.reset();
    robot = RobotIO::loadRobot(robotFile);

    if (!robot)
    {
        VR_ERROR << " no robot at " << robotFile << endl;
        return;
    }

    VR_INFO << "Searching EEF with name " << eefName << endl;
    eef = robot->getEndEffector(eefName);

    if (eef)
    {
        cout << "...OK" << endl;
    }
    else
    {
        cout << "...FAILED" << endl;
        return;
    }

    eefCloned =  eef->createEefRobot(eef->getName(), eef->getName());

    eef = eefCloned->getEndEffector(eef->getName());
    eef->getGCP()->showCoordinateSystem(true);

    if (!preshape.empty())
    {
        eef->setPreshape(preshape);
    }

    /*eefVisu->removeAllChildren();
    eefVisu->addChild(CoinVisualizationFactory::CreateEndEffectorVisualization(eef));*/
}

void MatGraspPlannerWindow::plan()
{
    //(MP 2013-09-30) Auskommentierung weg... (temp)


    //    float timeout = UI.spinBoxTimeOut->value() * 1000.0f;
    //    bool forceClosure = UI.checkBoxFoceClosure->isChecked();
    //    float quality = (float)UI.doubleSpinBoxQuality->value();
    //    int nrGrasps = UI.spinBoxGraspNumber->value();

    //    //MP create grasp planner
    //    //planner.reset(new GraspStudio::GenericGraspPlanner(grasps,qualityMeasure,approach,quality,forceClosure));
    //    planner.reset(new MatGraspPlanner(grasps,qualityMeasure,approach,quality,forceClosure));

    //    //MP plan grasps
    //    int nr = planner->plan(nrGrasps,timeout);
    //    VR_INFO << " Grasp planned:" << nr << endl;
    //    int start = (int)grasps->getSize()-nrGrasps-1;
    //    if (start<0)
    //        start = 0;
    //    grasps->setPreshape(preshape);

    //    //MP draw resulting grasps
    //    for (int i=start; i<(int)grasps->getSize()-1; i++)
    //    {
    //        //MP Tcp pose in world frame
    //        Eigen::Matrix4f m = grasps->getGrasp(i)->getTcpPoseGlobal(object->getGlobalPose());

    //        SoSeparator *sep1 = new SoSeparator();
    //        SoMatrixTransform *mt = CoinVisualizationFactory::getMatrixTransform(m);
    //        sep1->addChild(mt);
    //        sep1->addChild(eefVisu);
    //        graspsSep->addChild(sep1);
    //    }
    //    // set to last valid grasp
    //    if (grasps->getSize()>0 && eefCloned && eefCloned->getEndEffector(eefName))
    //    {
    //        Eigen::Matrix4f mGrasp = grasps->getGrasp(grasps->getSize()-1)->getTcpPoseGlobal(object->getGlobalPose());
    //        eefCloned->setGlobalPoseForRobotNode(eefCloned->getEndEffector(eefName)->getTcp(),mGrasp);
    //    }
    //    if (nrGrasps>0)
    //    {
    //        openEEF();
    //        closeEEF();
    //    }

}

/*
void MatGraspPlannerWindow::closeEEF(CandidateGraspPtr candidate, float positionScaleFactor)
{
#if 1
    static int pp=0;
    static Eigen::Vector3f approachDir;

//    object->getCollisionModel()->getTriMeshModel()->print();

//    if (pp==0)
//    {
//        Eigen::Vector3f position;
//        approach->getPositionOnObject(position,approachDir);

//        // set new pose
//        approach->setEEFToApproachPose(position,approachDir);
//        //eefCloned->getEndEffector(eefName)->getGCP()->showCoordinateSystem(true);


//    } else
//    {
//        //approach->moveEEFAway(approachDir,3.0f,5);
//        approach->moveEEFAway(candidate->handApproachDirection,3.0f,5); //MP 2013-10-01
//    }
//    pp = (pp+1) % 5;




    //=== MP 2013-10-01
    Eigen::Matrix4f poseGCP = candidate->toMatrix4f(positionScaleFactor);
    setEEF(poseGCP);

    cout << "todo: moveaway..." << endl;
    //approach->moveEEFAway(candidate->handApproachDirection,3.0f,5); //MP 2013-10-01
    //=== MP 2013-10-01


//  return;
#endif
    contacts.clear();

    float qual;
    if (eefCloned && eefCloned->getEndEffector(eefName))
    {
        contacts = eefCloned->getEndEffector(eefName)->closeActors(object);
        qualityMeasure->setContactPoints(contacts);

        qual = qualityMeasure->getGraspQuality();
        bool isFC = qualityMeasure->isGraspForceClosure();
        std::stringstream ss;
        ss << std::setprecision(3);
        ss << "Grasp Nr " << grasps->getSize() << "\nQuality (wrench space): "
           << qual << "\nForce closure: ";
        if (isFC)
            ss << "yes";
        else
            ss << "no";
        UI.labelInfo->setText(QString (ss.str().c_str()));
    }
    buildVisu();

    candidate->tested = true;
    candidate->qualityMindist = qual;
    //candidate->finalHandPose = eef->getGCP()->getGlobalPose(); //is that the correct pose?
}
*/

/*
void MatGraspPlannerWindow::openEEF()
{
    contacts.clear();
    if (eefCloned && eefCloned->getEndEffector(eefName))
    {
        eefCloned->getEndEffector(eefName)->openActors();
    }
    buildVisu();
}
*/

void MatGraspPlannerWindow::frictionConeVisu()
{
    buildVisu();
}

void MatGraspPlannerWindow::colModel()
{
    buildVisu();
}

void MatGraspPlannerWindow::showGrasps()
{
    buildVisu();
}


void MatGraspPlannerWindow::save()
{
    if (!object)
    {
        return;
    }

    ManipulationObjectPtr objectM(new ManipulationObject(object->getName(), object->getVisualization()->clone(), object->getCollisionModel()->clone()));
    objectM->addGraspSet(grasps);
    QString fi = QFileDialog::getSaveFileName(this, tr("Save ManipulationObject"), QString(), tr("XML Files (*.xml)"));
    objectFile = std::string(fi.toAscii());
    bool ok = false;

    try
    {
        ok = ObjectIO::saveManipulationObject(objectM, objectFile);
    }
    catch (VirtualRobotException& e)
    {
        cout << " ERROR while saving object" << endl;
        cout << e.what();
        return;
    }

    if (!ok)
    {
        cout << " ERROR while saving object" << endl;
        return;
    }
}

void MatGraspPlannerWindow::initPlanner()
{
    VR_INFO << "-------- Init MAT planner." << endl;

    if (!robot || !eef || !object || !qualityMeasure)
    {
        VR_ERROR << "no robot " << endl;
        return;
    }

    candidateTestCounter = 0;
    updateGraspPlannerConfigurationFromGui();

    std::string name = "MAT Grasp Planner - ";
    name += eef->getName();
    grasps.reset(new GraspSet(name, robot->getType(), eefName));

    planner.reset(new MatGraspPlanner(grasps, robot, eef, qualityMeasure,
                                      gpConfig, 0.0f, true,
                                      UI.checkBoxVerbose->isChecked()));

    planner->init();

    std::vector<MedialSpherePtr> medialSpheres = planner->getMedialSpheres();
    float maxSphereRadiusForColorComputation = SphereHelpers::findMaxSphereRadius(medialSpheres);

    drawMedialAxisPointCloud(medialSpheres, 0.5 * gpConfig->drawPointSize);
    drawMedialSpheres(medialSpheres, maxSphereRadiusForColorComputation);

    std::vector<MedialSpherePtr> medialSpheresFiltered = planner->getMedialSpheresFiltered();
    drawMedialAxisPointCloudFiltered(medialSpheresFiltered, 0.5 * gpConfig->drawPointSize);
    drawMedialSpheresFiltered(medialSpheresFiltered, maxSphereRadiusForColorComputation);

    std::vector<LocalNeighborhoodPtr> localNeighborhoods = planner->getLocalNeighborhoods();
    drawLocalNeighborhoods(localNeighborhoods, gpConfig, gpConfig->drawScale);
    drawSearchRadii(localNeighborhoods);
    std::vector<CandidateGraspPtr> candidateGraspsTemp = planner->getCandidateGrasps();
    drawCandidateGrasps(candidateGraspsTemp, 20 * gpConfig->drawScale);

    cout << "Generated " << planner->getCandidateGrasps().size() << " candidate grasps." << endl;
    candidateGrasps = planner->getCandidateGrasps();
    cout << "Everything done." << std::endl;
}

void MatGraspPlannerWindow::showNextGrasp()
{
    cout << "\nButton ShowGrasp pushed. Candidate number: "
         << candidateTestCounter << endl;

    if (candidateTestCounter >= (int)candidateGrasps.size())
    {
        cout << "No grasps available..." << endl;
        return;
    }

    CandidateGraspPtr currentCandidate = candidateGrasps.at(candidateTestCounter);

    simulateOneGrasp(currentCandidate);
    eefCloned->setGlobalPoseForRobotNode(eefCloned->getEndEffector(eefName)->getGCP(), currentCandidate->finalHandPose);
    eefCloned->setConfig(currentCandidate->finalJointAngles);

    candidateTestCounter++;

    if (candidateTestCounter >= (int)candidateGrasps.size())
    {
        candidateTestCounter = 0;
    }
}

void MatGraspPlannerWindow::testAllCandidates()
{
    //planner->cgTester->simulateAllGrasps(candidateGrasps);
}

/*
void MatGraspPlannerWindow::simulateAllGrasps(vector<CandidateGraspPtr> candidates)
{
    for (size_t i=0; i<candidates.size(); i++)
    {
        if (i % 10 == 0)
            cout << "Testing candidate " << i << " of " << candidates.size() << endl;
        simulateOneGrasp(candidates.at(i));
    }

    cout << "simulateAllGrasps: Done." << endl;
}
*/


void MatGraspPlannerWindow::simulateOneGrasp(CandidateGraspPtr candidate)
{
    cout << "candidate grasp BEFORE testing: " << endl;
    candidate->printDebug();

    // --- for visualization
    //int drawScaleCandidate = 40*gpConfig->drawScale;
    float drawScaleCandidate = 0.4f;         //ACHTUNG: als Argument...!
    drawStuffSep->removeAllChildren();
    SoSeparator* sepCandidate = DrawHelpers::DrawCandidateGrasp(candidate, drawScaleCandidate);
    drawStuffSep->addChild(sepCandidate);

    planner->testCandidate(candidate);

    cout << "candidate grasp AFTER testing: " << endl;
    candidate->printDebug();

}



void MatGraspPlannerWindow::removeStuff()
{
    printf("Button RemoveStuff pressed.\n");
    //drawStuffSep->removeChild(sep1);
    drawStuffSep->removeAllChildren();
}

void MatGraspPlannerWindow::testStuff()
{
    //temporary 2013-09-29 other stuff
    //TestCases::testSingleLinkedList();
    //TestCases::testTree();
    //TestCases::testFunStuff();
}




void MatGraspPlannerWindow::drawCandidateGrasp(CandidateGraspPtr cg, float drawScaleCandidate)
{
    SoSeparator* sepCandidate = DrawHelpers::DrawCandidateGrasp(cg, drawScaleCandidate);
    drawStuffSep->addChild(sepCandidate);
}

void MatGraspPlannerWindow::drawCandidateGrasps(std::vector<CandidateGraspPtr>& cg, float drawScaleCandidates)
{
    //                        cout << "candidate no. " << j << endl;
    //                        candidates.at(j).printDebug();
    //                        cout << "Now draw it..." << endl;
    SoSeparator* sepCandidates = DrawHelpers::DrawCandidateGrasps(cg,
                                 drawScaleCandidates);
    //drawStuffSep->addChild(sepCandidates);

    candidateGraspsSep->removeAllChildren();
    candidateGraspsSep->addChild(sepCandidates);
}

void MatGraspPlannerWindow::drawSearchRadius(LocalNeighborhoodPtr neighborhood)
{
    SoSeparator* sepSearchRadius = DrawHelpers::DrawSearchRadius(neighborhood);

    searchRadiusSep->addChild(sepSearchRadius);
}

void MatGraspPlannerWindow::drawSearchRadii(std::vector<LocalNeighborhoodPtr>& neighborhoods)
{
    SoSeparator* sepSearchRadii = DrawHelpers::DrawSearchRadii(neighborhoods);

    searchRadiusSep->removeAllChildren();
    searchRadiusSep->addChild(sepSearchRadii);
}

void MatGraspPlannerWindow::drawLocalNeighborhood(LocalNeighborhoodPtr neighborhood, GraspPlannerConfigurationPtr gpConfig, float drawScaleNeighborhood)
{
    //cout << "MatGraspPlannerWindow::drawLocalNeighborhood() called. "<< endl;
    //gpConfig->printDebug();

    SoSeparator* sepNeighborhood = DrawHelpers::DrawLocalNeighborhood(
                                       neighborhood,
                                       gpConfig->drawNeighborhoodEigenvectors,
                                       gpConfig->drawNeighborhoodCenterOfGravity,
                                       gpConfig->drawNeighborhoodSearchRadius,
                                       gpConfig->drawPointSize,
                                       drawScaleNeighborhood);
    //drawStuffSep->addChild(sepNeighborhood);

    //do not call removeChildren before, because we draw the neighborhoods incrementally,
    //only one at a time!
    neighborhoodSep->addChild(sepNeighborhood);
}


void MatGraspPlannerWindow::drawLocalNeighborhoods(std::vector<LocalNeighborhoodPtr>& neighborhoods, GraspPlannerConfigurationPtr gpConfig, float drawScaleNeighborhoods)
{
    SoSeparator* sepNeighborhoods = DrawHelpers::DrawLocalNeighborhoods(
                                        neighborhoods,
                                        gpConfig->drawNeighborhoodEigenvectors,
                                        gpConfig->drawNeighborhoodCenterOfGravity,
                                        gpConfig->drawNeighborhoodSearchRadius,
                                        gpConfig->drawPointSize,
                                        drawScaleNeighborhoods);
    //drawStuffSep->addChild(sepNeighborhoods);
    neighborhoodSep->removeAllChildren();
    neighborhoodSep->addChild(sepNeighborhoods);

}

void MatGraspPlannerWindow::drawSurfacePointCloud(float pointSize)
{
    float colorR = 0.0;
    float colorG = 0.0;
    float colorB = 0.0;
    float transparency = 0.0;
    std::vector<Eigen::Vector3f> surfacePoints = planner->getSurfacePoints();
    SoSeparator* sepSurfacePoints = DrawHelpers::DrawPointCloud(surfacePoints,
                                    colorR, colorG, colorB, transparency, pointSize);
    drawStuffSep->addChild(sepSurfacePoints);
}

void MatGraspPlannerWindow::drawMedialAxisPointCloud(
    std::vector<MedialSpherePtr>& spheres, float pointSize)
{
    //cout << "MatGraspPlannerWindow::drawMedialAxisPointCloud() called()." << endl;

    float colorR = 1.0;
    float colorG = 0.0;
    float colorB = 1.0;
    float transparency = 0.0;
    SoSeparator* sepMAPoints = DrawHelpers::DrawPointCloud(spheres,
                               colorR, colorG, colorB, transparency, pointSize);
    //drawStuffSep->addChild(sepMAPoints);

    medialAxisPointCloudSep->removeAllChildren(); //segfault -> warum?
    medialAxisPointCloudSep->addChild(sepMAPoints);
}

void MatGraspPlannerWindow::drawMedialAxisPointCloudFiltered(
    std::vector<MedialSpherePtr>& spheres, float pointSize)
{
    //cout << "MatGraspPlannerWindow::drawMedialAxisPointCloudFiltered() called()." << endl;

    float colorR = 1.0;
    float colorG = 0.0;
    float colorB = 1.0;
    float transparency = 0.0;
    SoSeparator* sepMAPoints = DrawHelpers::DrawPointCloud(spheres,
                               colorR, colorG, colorB, transparency, pointSize);
    //drawStuffSep->addChild(sepMAPoints);

    medialAxisPointCloudFilteredSep->removeAllChildren(); //segfault -> warum?
    medialAxisPointCloudFilteredSep->addChild(sepMAPoints);
}

void MatGraspPlannerWindow::drawMedialSpheres(std::vector<MedialSpherePtr>& spheres,
        float maxRadius)
{
    float transparency = 0.0;
    SoSeparator* sepMedialSpheres = DrawHelpers::DrawVectorOfSpheres(spheres,
                                    maxRadius, transparency);
    //drawStuffSep->addChild(sepMedialSpheres);
    medialSpheresSep->removeAllChildren();
    medialSpheresSep->addChild(sepMedialSpheres);
}

void MatGraspPlannerWindow::drawMedialSpheresFiltered(std::vector<MedialSpherePtr>& spheres,
        float maxRadius)
{
    float transparency = 0.0;
    SoSeparator* sepMedialSpheres = DrawHelpers::DrawVectorOfSpheres(spheres,
                                    maxRadius, transparency);
    //drawStuffSep->addChild(sepMedialSpheres);
    medialSpheresFilteredSep->removeAllChildren();
    medialSpheresFilteredSep->addChild(sepMedialSpheres);
}

/*
void MatGraspPlannerWindow::setEEF(Eigen::Matrix4f &poseGCP)
{
    if (!eefCloned)
        return;
    eefCloned->setGlobalPoseForRobotNode(eefCloned->getEndEffector(eefName)->getGCP(),poseGCP);
}
*/

void MatGraspPlannerWindow::updateGraspPlannerConfigurationFromGui()
{
    gpConfig->drawScale = UI.doubleSpinBoxDrawScale->value();
    gpConfig->drawPointSize = 0.3 * gpConfig->drawScale;

    gpConfig->neighborhoodSearchRadius =  UI.doubleSpinBoxSearchRadius->value();

    gpConfig->minimumSphereRadiusRelative =
        UI.doubleSpinBoxMinimumSphereRadiusRelative->value();

    gpConfig->numberOfApproachDirectionsForLocalSymmetryAxis =
        UI.spinBoxNumberOfApproachDirectionsForLocalSymmetryAxis->value();
    gpConfig->maxGraspDiameterOfCurrentRobotHand =
        UI.doubleSpinBoxMaxGraspDiameterOfCurrentRobotHand->value();

    gpConfig->fractionOfSpheresToAnalyze =
        UI.doubleSpinBoxFractionOfSpheresToAnalyze->value();
    gpConfig->stopAfterAnalyzingThisNumberOfSpheres =
        UI.spinBoxStopAfterAnalyzingThisNumberOfSpheres->value();

    gpConfig->printDebug();
}
/*
void MatGraspPlannerWindow::resetDataStructures()
{
    medialSpheres.clear();
    medialSpheresFiltered.clear();
    localNeighborhoods.clear();
    candidateGrasps.clear();
    candidateTestCounter = 0;
}*/

void MatGraspPlannerWindow::selectObject()
{
    //cout << "selectObject() called. " << endl;

    QString fi = QFileDialog::getOpenFileName(this, tr("Open Object File"), QString(), tr("XML Files (*.xml)"));
    //std::string s = m_sRobotFilename = std::string(fi.toAscii());
    std::string s = std::string(fi.toAscii());

    if (!s.empty())
    {
        if (UI.checkBoxVerbose->isChecked())
        {
            cout << "selectObject() Filename: " << s << endl;
        }

        loadObjectFromFile(s);

    }
    else if (UI.checkBoxVerbose->isChecked())
    {
        cout << "selectObject(): empty filenmame!" << endl;
    }
}





