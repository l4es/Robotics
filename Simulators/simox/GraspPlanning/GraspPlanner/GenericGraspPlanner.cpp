#include "GenericGraspPlanner.h"
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/RobotConfig.h>
#include <VirtualRobot/Nodes/RobotNode.h>
#include <iostream>
#include <sstream>
#include "../GraspQuality/GraspQualityMeasureWrenchSpace.h"
#include "../GraspQuality/GraspQualityMeasure.h"
#include "../ApproachMovementGenerator.h"

using namespace std;

namespace GraspStudio
{


    GenericGraspPlanner::GenericGraspPlanner(VirtualRobot::GraspSetPtr graspSet, GraspStudio::GraspQualityMeasurePtr graspQuality, GraspStudio::ApproachMovementGeneratorPtr approach, float minQuality, bool forceClosure)
        : GraspPlanner(graspSet), graspQuality(graspQuality), approach(approach), minQuality(minQuality), forceClosure(forceClosure)
    {
        THROW_VR_EXCEPTION_IF(!graspQuality, "NULL grasp quality...");
        THROW_VR_EXCEPTION_IF(!approach, "NULL approach...");
        THROW_VR_EXCEPTION_IF(!graspQuality->getObject(), "no object...");
        THROW_VR_EXCEPTION_IF(graspQuality->getObject() != approach->getObject(), "graspQuality and approach have to use the same object.");
        object = graspQuality->getObject();
        eef = approach->getEEF();
        THROW_VR_EXCEPTION_IF(!eef, "NULL eef in approach...");
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL graspSet...");
        verbose = true;
    }

    GenericGraspPlanner::~GenericGraspPlanner()
    {
    }

    int GenericGraspPlanner::plan(int nrGrasps, int timeOutMS, VirtualRobot::SceneObjectSetPtr obstacles)
    {
        startTime = clock();
        this->timeOutMS = timeOutMS;

        int nLoop = 0;
        int nGraspsCreated = 0;

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Searching " << nrGrasps << " grasps for EEF:" << approach->getEEF()->getName() << " and object:" << graspQuality->getObject()->getName() << ".\n";
            GRASPSTUDIO_INFO << ": Approach movements are generated with " << approach->getName() << endl;
            GRASPSTUDIO_INFO << ": Grasps are evaluated with " << graspQuality->getName() << endl;
        }

        while (!timeout() && nGraspsCreated < nrGrasps)
        {
            VirtualRobot::GraspPtr g = planGrasp(obstacles);

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

    VirtualRobot::GraspPtr GenericGraspPlanner::planGrasp(VirtualRobot::SceneObjectSetPtr obstacles)
    {

        std::string sGraspPlanner("Simox - GraspStudio - ");
        sGraspPlanner += graspQuality->getName();
        std::string sGraspNameBase = "Grasp ";

        VirtualRobot::RobotPtr robot = approach->getEEFOriginal()->getRobot();
        VirtualRobot::RobotNodePtr tcp = eef->getTcp();

        VR_ASSERT(robot);
        VR_ASSERT(tcp);


        bool bRes = approach->setEEFToRandomApproachPose();

        if (!bRes)
        {
            return VirtualRobot::GraspPtr();
        }

	if (obstacles)
        {
		VirtualRobot::CollisionCheckerPtr colChecker = eef->getCollisionChecker();
        VR_ASSERT(eef->getRobot());
        VR_ASSERT(obstacles);
	
		if (colChecker->checkCollision(eef->createSceneObjectSet(),obstacles)) {
//                GRASPSTUDIO_INFO << ": Collision detected before closing fingers" << endl;
            return VirtualRobot::GraspPtr();
}
        }

        VirtualRobot::EndEffector::ContactInfoVector contacts;
        contacts = eef->closeActors(object);

	if (obstacles)
        {
		VirtualRobot::CollisionCheckerPtr colChecker = eef->getCollisionChecker();
        VR_ASSERT(eef->getRobot());
        VR_ASSERT(obstacles);
		if (colChecker->checkCollision(eef->createSceneObjectSet(),obstacles)) {
  //              GRASPSTUDIO_INFO << ": Collision detected after closing fingers" << endl;
            return VirtualRobot::GraspPtr();
}
        }

        if (contacts.size() < 2)
        {
            if (verbose)
            {
                GRASPSTUDIO_INFO << ": ignoring grasp hypothesis, low number of contacts" << endl;
            }

            return VirtualRobot::GraspPtr();
        }

        graspQuality->setContactPoints(contacts);
        float score = graspQuality->getGraspQuality();

        if (score < minQuality)
        {
            return VirtualRobot::GraspPtr();
        }

        if (forceClosure && !graspQuality->isGraspForceClosure())
        {
            return VirtualRobot::GraspPtr();
        }

        // found valid grasp
        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Found grasp with " << contacts.size() << " contacts, score: " << score << endl;
        }

        std::stringstream ss;
        ss << sGraspNameBase << (graspSet->getSize() + 1);
        std::string sGraspName = ss.str();
        Eigen::Matrix4f objP = object->getGlobalPose();
        Eigen::Matrix4f pLocal = tcp->toLocalCoordinateSystem(objP);
        VirtualRobot::GraspPtr g(new VirtualRobot::Grasp(sGraspName, robot->getType(), eef->getName(), pLocal, sGraspPlanner, score));
        // set joint config
        VirtualRobot::RobotConfigPtr config = eef->getConfiguration();
        std::map< std::string, float > configValues = config->getRobotNodeJointValueMap();
        g->setConfiguration(configValues);
        return g;
    }

    bool GenericGraspPlanner::timeout()
    {
        if (timeOutMS <= 0)
        {
            return false;
        }

        clock_t endTime = clock();
        int timeMS = (int)(((float)(endTime - startTime) / (float)CLOCKS_PER_SEC) * 1000.0);
        return (timeMS > timeOutMS);
    }

} // namespace
