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
#include "MatGraspPlanner.h"
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <iostream>
#include <sstream>
#include <GraspPlanning/GraspQuality/GraspQualityMeasureWrenchSpace.h>
#include <GraspPlanning/GraspQuality/GraspQualityMeasure.h>
#include <GraspPlanning/ApproachMovementGenerator.h>

using namespace std;
using namespace VirtualRobot;

namespace GraspStudio
{


    MatGraspPlanner::MatGraspPlanner(VirtualRobot::GraspSetPtr graspSet,
                                     RobotPtr robot,
                                     EndEffectorPtr eef,
                                     GraspQualityMeasureWrenchSpacePtr graspQuality,
                                     GraspPlannerConfigurationPtr gpConfig,
                                     float minQuality,
                                     bool forceClosure,
                                     bool verbose)
        : GraspPlanner(graspSet), eef(eef), graspQuality(graspQuality), gpConfig(gpConfig), minQuality(minQuality), forceClosure(forceClosure)
    {
        THROW_VR_EXCEPTION_IF(!graspQuality, "NULL grasp quality...");
        THROW_VR_EXCEPTION_IF(!graspQuality->getObject(), "no object...");
        object = graspQuality->getObject();
        THROW_VR_EXCEPTION_IF(!eef, "NULL eef...");
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL graspSet...");

        this->robot = robot;

        eefName = eef->getName();


        if (!gpConfig)
        {
            // setup with standard parameters
            gpConfig.reset(new GraspPlannerConfiguration());
        }

        this->verbose = verbose;
        initialized = false;
    }

    MatGraspPlanner::~MatGraspPlanner()
    {
    }

    bool MatGraspPlanner::init()
    {
        // init powercrust data structures
        executePowercrust();

        if (pc->resultingPolarBalls.size() == 0)
        {
            VR_ERROR << "No polar balls from powercrust?!" << endl;
            return false;
        }

        cgGenerator = CandidateGraspGeneratorPtr(new CandidateGraspGenerator(verbose));

        cgTester = CandidateGraspTesterPtr(new CandidateGraspTester(robot, object,
                                           eefName, graspQuality, verbose));

        medialSpheres.clear();
        medialSpheresFiltered.clear();
        localNeighborhoods.clear();
        candidateGrasps.clear();

        //1. get medial spheres from powercrust
        medialSpheres = Converter::convertPowerCrustPolarBallsToMedialSpheres(
                            pc->resultingPolarBalls);

        float maxSphereRadiusForColorComputation =
            SphereHelpers::findMaxSphereRadius(medialSpheres);

        if (maxSphereRadiusForColorComputation <= 0)
        {
            VR_ERROR << "Max radius invalid?!" << endl;
            return false;
        }

        //drawMedialAxisPointCloud(medialSpheres, 0.5*gpConfig->drawPointSize);
        //drawMedialSpheres(medialSpheres, maxSphereRadiusForColorComputation);

        //2. filter spheres
        for (size_t i = 0; i < medialSpheres.size(); i++)
        {
            medialSpheresFiltered.push_back(medialSpheres.at(i)->clone());
        }

        SphereHelpers::filterSpheres(medialSpheresFiltered, gpConfig->minimumObjectAngle,
                                     gpConfig->minimumSphereRadiusRelative, verbose);

        //drawMedialAxisPointCloudFiltered(medialSpheresFiltered, 0.5*gpConfig->drawPointSize);
        //drawMedialSpheresFiltered(medialSpheresFiltered, maxSphereRadiusForColorComputation);

        //3. setup the grid
        goms.reset(new GridOfMedialSpheres(verbose));
        goms->setup(surfacePoints, medialSpheresFiltered, gpConfig->gridConstant);


        // compute candidates (todo: maybe we do not need to compute all candidates in advance, e.g. plan 10 grasps)
        computeAllCandidates();

        initialized = true;
        return true;
    }


    std::vector<CandidateGraspPtr> MatGraspPlanner::computeCandidates(MedialSpherePtr ms)
    {
        vector<CandidateGraspPtr> candidatesTemp;
        vector<MedialSpherePtr> neighborSpheres;
        neighborSpheres = goms->getSpheresInNeighborhood(
                              ms, gpConfig->neighborhoodSearchRadius);
        //        cout << "got " << neighborSpheres.size() << " spheres in neighborhood" << endl;

        LocalNeighborhoodPtr nbhd = LocalNeighborhoodPtr(new LocalNeighborhood(ms,
                                    neighborSpheres,
                                    gpConfig->neighborhoodSearchRadius));

        if (!nbhd->isEmpty)
        {
            nbhd->analyze();

            if (gpConfig->printNeighborhoods)
            {
                nbhd->printDebug();
            }

            if (nbhd->isValid())
            {
                localNeighborhoods.push_back(nbhd);

                candidatesTemp = cgGenerator->generateCandidateGrasps(nbhd, gpConfig);
            }
            else
            {
                //VR_INFO << "Invalid neighborhood for sphere ID: " << i << endl;
            }
        }
        else
        {
            if (verbose)
            {
                VR_INFO << "===> Local neighborhood empty, skip this sphere! If this occurs too often: maybe you should use a bigger search radius value?)" << endl;
            }
        }


        return candidatesTemp;
    }

    bool MatGraspPlanner::computeAllCandidates()
    {
        //4. analyze neighborhoods and generate candidate grasps
        candidateGrasps.clear();

        int sphereCounter = 0;
        int everyNthSphere = 1;

        if (gpConfig->fractionOfSpheresToAnalyze > 0)
        {
            everyNthSphere = (int)ceil(1.0 / (double)gpConfig->fractionOfSpheresToAnalyze);
        }

        for (size_t i = 0; i < medialSpheresFiltered.size(); i++)
        {
            if ((gpConfig->fractionOfSpheresToAnalyze < 1.0) && (i % everyNthSphere != 0))
            {
                //                cout << "Testing only every " << everyNthSphere
                //                     << "th sphere. Skipping... (fractionOfSpheresToAnalyze: "
                //                     << gpConfig->fractionOfSpheresToAnalyze << ") "
                //                     << endl;
                continue;
            }

            sphereCounter++;

            if (verbose)
            {
                if (i % 100 == 0)
                {
                    cout << "--- --- ---> sphere ID: " << i
                         << "; analyzing sphere no. " << sphereCounter
                         << " of " << medialSpheresFiltered.size()
                         << ", will stop after analyzing "
                         << gpConfig->stopAfterAnalyzingThisNumberOfSpheres << " spheres."
                         << endl;
                }
            }

            MedialSpherePtr ms = medialSpheresFiltered.at(i);
            //ms.printDebug();
            vector<CandidateGraspPtr> candidatesTemp = computeCandidates(ms);

            if (candidatesTemp.size() > 0)
            {
                vector<CandidateGraspPtr>::iterator itStart = candidatesTemp.begin();
                vector<CandidateGraspPtr>::iterator itEnd = candidatesTemp.end();
                candidateGrasps.insert(candidateGrasps.end(), itStart, itEnd);

                if (gpConfig->printCandidateGrasps && verbose)
                {
                    for (size_t k = 0; k < candidatesTemp.size(); k++)
                    {
                        candidatesTemp.at(k)->printDebug();
                    }
                }
            }

            if ((gpConfig->stopAfterAnalyzingThisNumberOfSpheres > 0) &&
                (sphereCounter > gpConfig->stopAfterAnalyzingThisNumberOfSpheres))
            {
                if (verbose)
                {

                    VR_INFO << "Stopping after analyzing " << sphereCounter
                            << " medial spheres. " << std::endl;
                }

                break;
            }
        }

        return (candidateGrasps.size() > 0);
    }

    int MatGraspPlanner::plan(int nrGrasps, int timeOutMS, VirtualRobot::SceneObjectSetPtr obstacles)
    {
        if (!initialized)
        {
            init();
        }

        startTime = clock();
        this->timeOutMS = timeOutMS;

        int nLoop = 0;
        int nGraspsCreated = 0;

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Searching " << nrGrasps << " grasps for EEF:" << eef->getName() << " and object:" << graspQuality->getObject()->getName() << ".\n";
            GRASPSTUDIO_INFO << ": Grasps are evaluated with " << graspQuality->getName() << endl;
        }

        while (!timeout() && nGraspsCreated < nrGrasps)
        {
            VirtualRobot::GraspPtr g = planGrasp();

            if (g)
            {
                if (graspSet)
                {
                    graspSet->addGrasp(g);
                }

                plannedGrasps.push_back(g);
                nGraspsCreated++;
            }

            nLoop++;
        }

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": created " << nGraspsCreated << " valid grasps in " << nLoop << " loops" << endl;
        }

        return nGraspsCreated;
    }

    VirtualRobot::GraspPtr MatGraspPlanner::planGrasp()
    {
        /*
        std::string sGraspPlanner("Simox - GraspStudio - ");
        sGraspPlanner += graspQuality->getName();
        std::string sGraspNameBase = "Grasp ";

        VirtualRobot::RobotPtr robot = approach->getEEFOriginal()->getRobot();
        VirtualRobot::RobotNodePtr tcp = eef->getTcp();

        VR_ASSERT(robot);
        VR_ASSERT(tcp);


        bool bRes = approach->setEEFToRandomApproachPose();
        if (!bRes)
            return VirtualRobot::GraspPtr();

        VirtualRobot::EndEffector::ContactInfoVector contacts;
        contacts = eef->closeActors(object);

        if (contacts.size()<2)
        {
            if (verbose)
                GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, low number of contacts" << endl;
            return VirtualRobot::GraspPtr();
        }

        graspQuality->setContactPoints(contacts);
        float score = graspQuality->getGraspQuality();
        if (score<minQuality)
            return VirtualRobot::GraspPtr();
        if (forceClosure && !graspQuality->isGraspForceClosure())
            return VirtualRobot::GraspPtr();

        // found valid grasp
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Found grasp with " << contacts.size() << " contacts, score: " << score << endl;
        }
        std::stringstream ss;
        ss << sGraspNameBase << (graspSet->getSize()+1);
        std::string sGraspName = ss.str();
        Eigen::Matrix4f objP = object->getGlobalPose();
        Eigen::Matrix4f pLocal = tcp->toLocalCoordinateSystem(objP);
        VirtualRobot::GraspPtr g(new VirtualRobot::Grasp(sGraspName,robot->getType(),eef->getName(),pLocal,sGraspPlanner,score));
        // set joint config
        VirtualRobot::RobotConfigPtr config = eef->getConfiguration();
        std::map< std::string, float > configValues = config->getRobotNodeJointValueMap();
        g->setConfiguration(configValues);
        return g;
        */
        VirtualRobot::GraspPtr gp;
        return gp;
    }

    bool MatGraspPlanner::timeout()
    {
        if (timeOutMS <= 0)
        {
            return false;
        }

        clock_t endTime = clock();
        int timeMS = (int)(((float)(endTime - startTime) / (float)CLOCKS_PER_SEC) * 1000.0);
        return (timeMS > timeOutMS);
    }



    void MatGraspPlanner::executePowercrust()
    {
        surfacePoints.clear();
        surfacePoints = getSurfacePoints(0.001f); // convert from MM to M

        if (verbose)
        {
            VR_INFO << "Nr of points: " << surfacePoints.size() << endl;
        }

        pc.reset(new PowerCrust::PowerCrust());
        pc->setVerbose(verbose);
        pc->init(surfacePoints);
        pc->computeMedialAxis();
        size_t numPolarBalls = pc->resultingPolarBalls.size();

        if (verbose)
        {
            VR_INFO << "MAT computation complete. Number of resulting polar balls: " << numPolarBalls << endl;
        }
    }

    std::vector<Eigen::Vector3f> MatGraspPlanner::getSurfacePoints(float scaling, bool checkForDoubledEntries)
    {
        std::vector<Eigen::Vector3f> res;

        if (!object || !object->getCollisionModel() || !object->getCollisionModel()->getTriMeshModel())
        {
            VR_ERROR << "No collision model..." << endl;
            return res;
        }

        TriMeshModelPtr tm = object->getCollisionModel()->getTriMeshModel();

        for (size_t i = 0; i < tm->vertices.size(); i++)
        {
            bool entryFound = false;

            if (checkForDoubledEntries)
                for (size_t j = 0; j < res.size(); j++)
                {
                    if (res[j] == tm->vertices[i])
                    {
                        entryFound = true;
                        break;
                    }
                }

            if (!entryFound)
            {
                res.push_back(tm->vertices[i]*scaling);
            }
        }

        return res;
    }


    std::vector<MedialSpherePtr> MatGraspPlanner::getMedialSpheres()
    {
        return medialSpheres;
    }

    std::vector<MedialSpherePtr> MatGraspPlanner::getMedialSpheresFiltered()
    {
        return medialSpheresFiltered;
    }

    std::vector<LocalNeighborhoodPtr> MatGraspPlanner::getLocalNeighborhoods()
    {
        return localNeighborhoods;
    }

    std::vector<CandidateGraspPtr> MatGraspPlanner::getCandidateGrasps()
    {
        return candidateGrasps;
    }

    std::vector<Eigen::Vector3f> MatGraspPlanner::getSurfacePoints()
    {
        return surfacePoints;
    }

    void MatGraspPlanner::testCandidate(CandidateGraspPtr candidate)
    {
        if (!candidate || !cgTester)
        {
            return;
        }

        cgTester->testCandidate(candidate);
    }

} // namespace
