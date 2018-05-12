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
#ifndef __MatGraspPlanner_WINDOW_H_
#define __MatGraspPlanner_WINDOW_H_

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <VirtualRobot/Visualization/VisualizationFactory.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Obstacle.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Obstacle.h>

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

#include "ui_MatGraspPlanner.h"

#include "GraspPlanning/GraspPlanner/MATPlanner/TestCases.h"
#include "GraspPlanning/GraspPlanner/MATPlanner/StrOutHelpers.h"
#include "GraspPlanning/GraspStudio.h"
#include "GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include "GraspPlanning/GraspPlanner/MATPlanner/MatGraspPlanner.h"
#include "GraspPlanning/GraspPlanner/MATPlanner/MeshConverter.h"
#include "GraspPlanning/GraspPlanner/MATPlanner/GraspPlannerConfiguration.h"

class MatGraspPlannerWindow : public QMainWindow
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Q_OBJECT
public:
    MatGraspPlannerWindow(std::string& robotFile, std::string& eefName, std::string& preshape, std::string& objectFile, Qt::WFlags flags = 0);
    ~MatGraspPlannerWindow();

    /*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
    int main();

public slots:
    /*! Closes the window and exits SoQt runloop. */
    void quit();

    /*!< Overriding the close event, so we know when the window was closed by the user. */
    void closeEvent(QCloseEvent* event);

    void resetSceneryAll();

    void colModel();
    void frictionConeVisu();
    void showGrasps();

    void showNextGrasp();

    void buildVisu();

    void plan();
    void save();

    void initPlanner();

    void removeStuff();

    //Helpers
    void drawCandidateGrasp(GraspStudio::CandidateGraspPtr cg, float drawScaleCandidate);
    void drawCandidateGrasps(std::vector<GraspStudio::CandidateGraspPtr>& cg, float drawScaleCandidates);
    void drawLocalNeighborhood(GraspStudio::LocalNeighborhoodPtr nbhds,
                               GraspStudio::GraspPlannerConfigurationPtr gpConfig,
                               float drawScaleNeighborhoods);
    void drawLocalNeighborhoods(std::vector<GraspStudio::LocalNeighborhoodPtr>& nbhds,
                                GraspStudio::GraspPlannerConfigurationPtr gpConfig,
                                float drawScaleNeighborhoods);
    void drawSurfacePointCloud(float pointSize);
    void drawMedialAxisPointCloud(std::vector<GraspStudio::MedialSpherePtr>& spheres,
                                  float pointSize);
    void drawMedialSpheres(std::vector<GraspStudio::MedialSpherePtr>& spheres, float maxRadius);
    void drawMedialAxisPointCloudFiltered(std::vector<GraspStudio::MedialSpherePtr>& spheres,
                                          float pointSize);
    void drawMedialSpheresFiltered(std::vector<GraspStudio::MedialSpherePtr>& spheres, float maxRadius);
    void drawSearchRadius(GraspStudio::LocalNeighborhoodPtr neighborhood);
    void drawSearchRadii(std::vector<GraspStudio::LocalNeighborhoodPtr>& neighborhood);


    void testStuff();

    void testAllCandidates(); //GUI


    void updateGraspPlannerConfigurationFromGui();

    void selectObject();

protected:

    std::vector<Eigen::Vector3f> getSurfacePoints(float scaling = 1.0f, bool checkForDoubledEntries = false);

    void loadRobot();
    void loadObject();
    void loadObjectFromFile(std::string objectFilename);

    void simulateOneGrasp(GraspStudio::CandidateGraspPtr candidate);

    void setupUI();

    static void timerCB(void* data, SoSensor* sensor);
    Ui::MatGraspPlanner UI;
    SoQtExaminerViewer* viewer; /*!< Viewer to display the 3D model of the robot and the environment. */

    SoSeparator* sceneSep;
    SoSeparator* eefClonedSep;
    SoSeparator* objectSep;
    SoSeparator* frictionConeSep;
    SoSeparator* graspsSep;

    SoSeparator* drawStuffSep;
    SoSeparator* drawStuffGuiSep;
    SoSeparator* medialAxisPointCloudSep;
    SoSeparator* medialSpheresSep;
    SoSeparator* medialAxisPointCloudFilteredSep;
    SoSeparator* medialSpheresFilteredSep;
    SoSeparator* neighborhoodSep;
    SoSeparator* searchRadiusSep;
    SoSeparator* candidateGraspsSep;

    VirtualRobot::RobotPtr robot;
    VirtualRobot::RobotPtr eefCloned;
    VirtualRobot::ObstaclePtr object;
    VirtualRobot::EndEffectorPtr eef;

    VirtualRobot::GraspSetPtr grasps;


    VirtualRobot::EndEffector::ContactInfoVector contacts;


    std::string robotFile;
    std::string objectFile;
    std::string eefName;
    std::string preshape;

    SoSeparator* eefVisu;

    GraspStudio::GraspQualityMeasureWrenchSpacePtr qualityMeasure;
    GraspStudio::MatGraspPlannerPtr planner;

    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationEEFCloned;
    boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject;

    GraspStudio::GraspPlannerConfigurationPtr gpConfig;
    int candidateTestCounter;
    std::vector<GraspStudio::CandidateGraspPtr> candidateGrasps;

};

#endif // __MatGraspPlanner_WINDOW_H_
