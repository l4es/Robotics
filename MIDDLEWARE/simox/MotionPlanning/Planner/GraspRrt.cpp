#include "GraspRrt.h"

#include <VirtualRobot/MathTools.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/EndEffector/EndEffectorActor.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/Grasping/BasicGraspQualityMeasure.h>
#include <algorithm>
#include <float.h>
#include <time.h>

using namespace std;

namespace Saba
{


    GraspRrt::GraspRrt(CSpaceSampledPtr cspace,
                       VirtualRobot::EndEffectorPtr eef,
                       VirtualRobot::ObstaclePtr object,
                       VirtualRobot::BasicGraspQualityMeasurePtr measure,
                       VirtualRobot::SceneObjectSetPtr graspCollisionObjects,
                       float probabGraspHypothesis,
                       float graspQualityMinScore
                      )
        : Rrt(cspace), probabGraspHypothesis(probabGraspHypothesis), graspQualityMinScore(graspQualityMinScore)
    {
        plannerInitialized = false;
		minGraspContacts = 3;
        name = "GraspRrt";
        THROW_VR_EXCEPTION_IF(!cspace || !object || !eef || !cspace->getRobotNodeSet() || !cspace->getRobot() || !measure, "NULL data");
        rns = cspace->getRobotNodeSet();
        robot = cspace->getRobot();
        targetObject = object;
        this->eef = eef;
        this->graspQualityMeasure = measure;

        if (graspCollisionObjects)
        {
            this->graspCollisionObjects = graspCollisionObjects->clone();
        }
        else
        {
            this->graspCollisionObjects.reset(new VirtualRobot::SceneObjectSet("GraspRrtSceneObject", object->getCollisionChecker()));
        }

        if (!this->graspCollisionObjects->hasSceneObject(targetObject))
        {
            this->graspCollisionObjects->addSceneObject(targetObject);
        }



        cartSamplingPosStepSize = 10.0f;
        cartSamplingOriStepSize = (float)M_PI / 18.0f;  // == 10 degree

        THROW_VR_EXCEPTION_IF(dimension <= 0, "Zero dim?");
        THROW_VR_EXCEPTION_IF(!eef->getGCP(), "No gcp..");

        performanceMeasure.setupTimeMS = 0.0f;
        performanceMeasure.planningTimeMS = 0.0f;
        performanceMeasure.moveTowardGraspPosesTimeMS = 0.0f;
        performanceMeasure.numberOfMovementsTowardGraspPose = 0;
        performanceMeasure.scoreGraspTimeMS = 0.0f;
        performanceMeasure.numberOfGraspScorings = 0;
        performanceMeasure.rrtTimeMS = 0.0f;
        foundSolution = false;
        verbose = false;
#ifdef _DEBUG
        verbose = true;
#endif
        targetObjectPosition = targetObject->getGlobalPose().block(0, 3, 3, 1);
        // todo: is this value a good guess?
        tryGraspsDistance2 = 600.0f * 600.0f;

        gcpOject = VirtualRobot::Obstacle::createBox(1.0f, 1.0f, 1.0f, VirtualRobot::VisualizationFactory::Color::Red(), "", eef->getCollisionChecker());
    }

    GraspRrt::~GraspRrt()
    {

    }


    void GraspRrt::reset()
    {
        Rrt::reset();

        if (poseSphere)
        {
            poseSphere.reset();
        }

        performanceMeasure.setupTimeMS = 0.0f;
        performanceMeasure.planningTimeMS = 0.0f;
        performanceMeasure.moveTowardGraspPosesTimeMS = 0.0f;
        performanceMeasure.numberOfMovementsTowardGraspPose = 0;
        performanceMeasure.scoreGraspTimeMS = 0.0f;
        performanceMeasure.numberOfGraspScorings = 0;
        performanceMeasure.rrtTimeMS = 0.0f;
        plannerInitialized = false;
        foundSolution = false;
        colChecksOverall = 0;

        mapConfigTcp.clear();
    }

    bool GraspRrt::init()
    {
        performanceMeasure.setupTimeMS = 0.0f;
        performanceMeasure.planningTimeMS = 0.0f;
        performanceMeasure.moveTowardGraspPosesTimeMS = 0.0f;
        performanceMeasure.numberOfMovementsTowardGraspPose = 0;
        performanceMeasure.scoreGraspTimeMS = 0.0f;
        performanceMeasure.numberOfGraspScorings = 0;
        performanceMeasure.rrtTimeMS = 0.0f;
        foundSolution = false;
        colChecksOverall = 0;

        clock_t timeStart = clock();
        solution.reset();

        if (!startNode)
        {
            VR_ERROR << ": not initialized correctly..." << endl;
            return false;
        }

        cycles = 0;

        stopSearch = false;

        workSpaceSamplingCount = 0;

        grasps.clear();

        poseSphere.reset(new ApproachDiscretization());

        poseSphere->setGlobalPose(targetObject->getGlobalPose());

        targetObjectPosition = targetObject->getGlobalPose().block(0, 3, 3, 1);

        // init jacobian solver

        diffIK.reset(new VirtualRobot::DifferentialIK(rns));

        plannerInitialized = true;
        clock_t timeEnd = clock();
        performanceMeasure.setupTimeMS = ((float)(timeEnd - timeStart) / (float)CLOCKS_PER_SEC) * 1000.0f;
        return true;
    }

    bool GraspRrt::doPlanningCycle()
    {
        static const float randMult = (float)(1.0 / (double)(RAND_MAX));

        if (!plannerInitialized)
        {
            return false;
        }

        int colChecksStart = cspace->performaceVars_collisionCheck;

        // CHOOSE A RANDOM CONFIGURATION
        // check if we want to go to the goal directly or extend randomly
        bool bDoNormalConnect = true;
        float r = (float)rand() * randMult;

        if (r <= probabGraspHypothesis)
        {
            bDoNormalConnect = false;

            extendGraspStatus = connectRandomGraspPositionJacobian();

            switch (extendGraspStatus)
            {
                case eFatalError:
                    stopSearch = true;
                    break;

                case eGraspablePoseReached:
                    cout << endl << __FUNCTION__ << " -- FOUND A GRASPABLE POSITION (but score is not high enough) -- " << endl;

                    if (grasps.size() <= 0)
                    {
                        cout << __FUNCTION__ << ": Error, no grasp results stored..." << endl;
                    }
                    else
                    {
                        GraspInfo graspInfo = grasps[grasps.size() - 1];
                        printGraspInfo(graspInfo);
                    }

                    break;

                case eGoalReached:
                    cout << endl << __FUNCTION__ << " -- FOUND GOAL PATH (CONNECTION TO GRASPING POSITION VIA INV JACOBIAN) -- " << endl;

                    if (grasps.size() <= 0)
                    {
                        cout << __FUNCTION__ << ": Error, no grasp results stored..." << endl;
                        stopSearch = true;

                    }
                    else
                    {
                        GraspInfo graspInfo = grasps[grasps.size() - 1];
                        printGraspInfo(graspInfo);
                        goalNode = tree->getNode(graspInfo.rrtNodeId);
                        foundSolution = true;
                    }

                    break;

                default:
                    // nothing to to
                    break;
            }
        }

        if (bDoNormalConnect)
        {
            cspace->getRandomConfig(tmpConfig);
            ExtensionResult extendStatus = connectComplete(tmpConfig, tree, lastAddedID);

            if (extendStatus == Rrt::eError)
            {
                stopSearch = true;
            }
        }

        cycles++;

        colChecksOverall += cspace->performaceVars_collisionCheck - colChecksStart;

        return true;
    }

    Rrt::ExtensionResult GraspRrt::connectComplete(Eigen::VectorXf& c, CSpaceTreePtr tree, int& storeLastAddedID)
    {
        // NEAREST NEIGHBOR OF RANDOM CONFIGURATION
        CSpaceNodePtr nn = tree->getNearestNeighbor(c);

        SABA_ASSERT(nn);

        // CHECK PATH FOR COLLISIONS AND VALID NODES
        if (!cspace->isPathValid(nn->configuration, c))
        {
            return eFailed; // CONNECT FAILS
        }

        // ADD IT TO RRT TREE
        if (!tree->appendPath(nn, c, &storeLastAddedID))
        {
            return Rrt::eError;
        }

        // add nodes to pose sphere and store TCP poses
        processNodes(nn->ID, storeLastAddedID);

        return Rrt::eSuccess; // REACHED
    }


    bool GraspRrt::plan(bool bQuiet)
    {
        if (!plannerInitialized)
            if (!init())
            {
                return false;
            }

        cspace->getRobot()->setUpdateVisualization(false);

        clock_t startClock = clock();
        time_t startTime = time(NULL);

        bool bStopLoop = false;

        // the planning loop
        do
        {
            doPlanningCycle();

            if (stopSearch)
            {
                bStopLoop = true;
            }

            //if (!m_bEndlessMode)
            //{
            if (foundSolution)
            {
                bStopLoop = true;
            }

            //}
            if (cycles > maxCycles)
            {
                bStopLoop = true;
            }
        }
        while (!bStopLoop);

        time_t endTime = time(NULL);
        clock_t endClock = clock();

        //long diffClock = (long)(((float)(endClock - startClock) / (float)CLOCKS_PER_SEC) * 1000.0);
        //long diffTime = (long)((float)(endTime - startTime) * 1000.0);
        performanceMeasure.planningTimeMS = ((float)(endClock - startClock) / (float)CLOCKS_PER_SEC) * 1000.0f;
        planningTime = performanceMeasure.planningTimeMS;


        // calculate the RRT buildup time (the time for moving toward the goals includes the scoring time)
        performanceMeasure.rrtTimeMS = performanceMeasure.planningTimeMS - performanceMeasure.moveTowardGraspPosesTimeMS;
        // until now the time for moving toward goal includes the grasp scoring time, so subtract it
        performanceMeasure.moveTowardGraspPosesTimeMS -= performanceMeasure.scoreGraspTimeMS;

        if (!bQuiet)
        {
            printPerformanceResults();
        }

        cspace->getRobot()->setUpdateVisualization(true);

        if (foundSolution)
        {
            if (!bQuiet)
            {
                SABA_INFO << "Found path in rrt with " << cycles << " cycles." << std::endl;
            }

            return createSolution();
        }

        // something went wrong...
        if (cycles >= maxCycles)
        {
            if (!bQuiet)
            {
                SABA_ERROR << " maxCycles exceeded..." << std::endl;
            }
        }

        if (stopSearch)
        {
            if (!bQuiet)
            {
                SABA_ERROR << " search was stopped..." << std::endl;
            }
        }

        return false;
    }

    void GraspRrt::printPerformanceResults()
    {
        cout << endl << "=================== RESULTS ===================" << endl;
        std::cout << "Needed " << performanceMeasure.setupTimeMS << " ms for setup." << std::endl;
        std::cout << "Needed " << performanceMeasure.planningTimeMS << " ms for the complete planning process." << std::endl;

        std::cout << "Needed " << performanceMeasure.rrtTimeMS << " ms for building up the RRT." << std::endl;
        std::cout << "Needed " << performanceMeasure.moveTowardGraspPosesTimeMS << " ms for moving toward the grasp goals." << std::endl;
        std::cout << "Needed " << performanceMeasure.scoreGraspTimeMS << " ms for scoring the grasps." << std::endl;
        cout << "  Nr of grasp tries: " << performanceMeasure.numberOfMovementsTowardGraspPose << endl;

        if (performanceMeasure.numberOfMovementsTowardGraspPose > 0)
        {
            cout << "  Avg Time for moving to grasping test position (without checking the grasp score):" << performanceMeasure.moveTowardGraspPosesTimeMS / (float)performanceMeasure.numberOfMovementsTowardGraspPose << " ms " << endl;
        }

        cout << "  Nr of grasp scorings: " << performanceMeasure.numberOfGraspScorings << endl;

        if (performanceMeasure.numberOfGraspScorings > 0)
        {
            cout << "  Avg Time for scoring a grasp:" << performanceMeasure.scoreGraspTimeMS / (float)performanceMeasure.numberOfGraspScorings << " ms " << endl;
        }

        std::cout << "Created " << tree->getNrOfNodes() << " nodes." << std::endl;
        std::cout << "Collision Checks: " << colChecksOverall << std::endl;
        cout << "=================== RESULTS ===================" << endl << endl;
    }

    bool GraspRrt::setStart(const Eigen::VectorXf& startVec)
    {
        if (!tree)
        {
            return false;
        }

        if (!Rrt::setStart(startVec) || !startNode)
        {
            SABA_ERROR << " error initializing start node..." << endl;
            return false;
        }

        startNode->status = 0;
        processNode(startNode);

        if (verbose)
        {
            cout << "GraspRrt::setStart" << endl;
            cout << "startVec:";

            for (unsigned int i = 0; i < dimension; i++)
            {
                cout << startNode->configuration[i] << ",";
            }

            cout <<  endl;
        }

        return true;
    }


    bool GraspRrt::setGoal(const Eigen::VectorXf& c)
    {
        THROW_VR_EXCEPTION("Not allowed here, goal configurations are sampled during planning..");
        return false;
    }


    bool GraspRrt::calculateGlobalGraspPose(const Eigen::VectorXf& c, Eigen::Matrix4f& storeGoal)
    {
        Eigen::Vector3f P1;
        Eigen::Vector3f P2;
        int nId1, nId2;
        Eigen::Matrix4f mMat, mRotMat;


        // set gcp object
        robot->setJointValues(rns, c);
        gcpOject->setGlobalPose(eef->getGCP()->getGlobalPose());

        // get target position (position on grasp object with shortest distance to hand)
        double dist = targetObject->getCollisionChecker()->calculateDistance(targetObject->getCollisionModel(), gcpOject->getCollisionModel(), P1, P2, &nId1, &nId2);

        // now target position in global coord system is stored in P1

        // calculate target orientation

        // a) get P1 in local grasping position coordinate system
        mMat.setIdentity();
        mMat.block(0, 3, 3, 1) = P1;
        mMat = eef->getGCP()->toLocalCoordinateSystem(mMat);

        Eigen::Vector3f vecTarget_local;
        Eigen::Vector3f vecZ_local;
        vecTarget_local = mMat.block(0, 3, 3, 1);
        vecZ_local[0] = 0.0f;
        vecZ_local[1] = 0.0f;
        vecZ_local[2] = 1.0f;

        //b) get angle between z axis of grasping position (vecZ_local) and target distance vector (vecTarget_local)
        float l = vecTarget_local.norm();

        if (l < 1e-8)
        {
            cout << __FUNCTION__ << ":WARNING: length to target is small ... aborting " << endl;
            return false;
        }

        vecTarget_local.normalize();
        float fAngle = VirtualRobot::MathTools::getAngle(vecTarget_local, vecZ_local);
        //cout << "Angle : " << fAngle << endl;

        // c) rotation axis (is orthogonal on vecZ_local and vecTarget_local)
        Eigen::Vector3f rotAxis_local;
        rotAxis_local = vecZ_local.cross(vecTarget_local);

        mRotMat = VirtualRobot::MathTools::axisangle2eigen4f(rotAxis_local, fAngle);

        // construct global grasp pose
        mMat = eef->getGCP()->toGlobalCoordinateSystem(mRotMat);
        mMat.block(0, 3, 3, 1) = P1;

        storeGoal = mMat;
        return true;
    }

    GraspRrt::MoveArmResult GraspRrt::connectRandomGraspPositionJacobian()
    {
        clock_t timeStart = clock();
        // choose one node to extend
        /*Eigen::Matrix4f mObjPose = *(m_pManipulationObject->GetGlobalPose());
        MathTools::Eigen::Matrix4f2PosQuat(mObjPose, m_pTmpPose);
        CSpaceNode *nnNode = findNearestNeighborCart(m_pTmpPose);

        if (nnNode == NULL)
            return eError;

        // mark nn node, so it will never be selected again in findNearestNeighborCart()
        nnNode->failurecount++;*/

        CSpaceNodePtr nnNode = poseSphere->getGoodRatedNode();

        if (!nnNode)
        {
            cout << __FUNCTION__ << ": No good ranked node found..." << endl;
            return eTrapped;
        }

        // set config and compute target pos on object's surface
        Eigen::Matrix4f goalPose;

        if (!calculateGlobalGraspPose(nnNode->configuration, goalPose))
        {
            return eError;
        }

        //MathTools::Eigen::Matrix4f2PosQuat(goalPose,m_pTmpPose);

        //cout << "GoalGrasp:" << grasp->GetName() << endl;
        //cout << "pos:" << m_pTmpPose[0] << "," << m_pTmpPose[1] << "," << m_pTmpPose[2] << endl;
        GraspRrt::MoveArmResult res = moveTowardsGoal(nnNode, goalPose, 300);
        clock_t timeEnd = clock();
        float fMoveTime = ((float)(timeEnd - timeStart) / (float)CLOCKS_PER_SEC) * 1000.0f;
        performanceMeasure.moveTowardGraspPosesTimeMS += fMoveTime;
        performanceMeasure.numberOfMovementsTowardGraspPose++;

        if (verbose)
        {
            cout << __FUNCTION__ << " - Time for Connect To Grasp Position:" << fMoveTime << endl;
        }

        return res;

    }

    // not needed any more
    /*CSpaceNode* GraspRrt::findNearestNeighborCart(float *pCartConf)
    {
        if (!pCartConf || !m_pCSpace)
            return NULL;

        float minDist = FLT_MAX;
        float actDist;
        CSpaceNode* bestNode = NULL;
        std::vector<CSpaceNode*> *nodes = m_pRrtCart->getNodes();
        if (nodes->size()==0)
            return NULL;
        CSpaceNode *testNode;
        unsigned int loops = 400; // TODO: this is just a random number, better things should be possible?!
        for (unsigned int i=0; i<loops; i++)
        {
            int nr = rand()%((int)nodes->size());
            testNode = (*nodes)[nr];
            if (testNode->status==0)
            {
                actDist = MathHelpers::calcCartesianPoseDiff(testNode->cartPosTCP1,pCartConf);
                if (actDist < minDist)
                {
                    minDist = actDist;
                    bestNode = testNode;
                    return bestNode;
                }
            }
        }
        if (minDist==FLT_MAX)
        {
            minDist = MathHelpers::calcCartesianPoseDiff((*nodes)[0]->cartPosTCP1,pCartConf);
            bestNode = (*nodes)[0];
        }
        //cout << " , minDist: " << minDist ;
        return bestNode;
    }*/

    GraspRrt::MoveArmResult GraspRrt::moveTowardsGoal(CSpaceNodePtr startNode, const Eigen::Matrix4f& targetPose, int nMaxLoops)
    {
        if (!startNode)
        {
            return eError;
        }

        CSpaceNodePtr lastExtendNode = startNode;
        float dist;
        bool r;
        dist = VirtualRobot::MathTools::getCartesianPoseDiff(mapConfigTcp[lastExtendNode], targetPose);
        int loopCount = 0;
        float lastDist = 0;
        float fMaxDist_GraspGoalReached = 1.0f; // 1 mm or 3 degrees

        while (loopCount < nMaxLoops)
        {
            MoveArmResult res = createWorkSpaceSamplingStep(targetPose, lastExtendNode, tmpConfig);

            // check result
            switch (res)
            {
                // ERROR -> abort
                case eError:
                case eFatalError:
                    return res;
                    break;

                // COLLISION -> don't apply step, but try to close hand
                // todo: here just the eCollision_Environment is used,
                //    maybe it's better to check only the TCP collisions
                case eCollision_Environment:
                {
                    // check grasp score of last valid config
                    float fGraspScore = calculateGraspScore(lastExtendNode->configuration, lastExtendNode->ID, true);

                    if (fGraspScore >= graspQualityMinScore)
                    {
                        cout << __FUNCTION__ << ": found valid grasp (score:" << fGraspScore << ")" << endl;
                        return eGoalReached;
                    }
                    else if (fGraspScore > 0.0f)
                    {
                        cout << __FUNCTION__ << ": found valid grasp (score:" << fGraspScore << ")" << endl;
                        return eGraspablePoseReached;
                    }
                    else
                    {
                        if (verbose)
                        {
                            cout << __FUNCTION__ << ": aborting moveArmToGraspPos, Collision but no graspable config " << endl;
                        }

                        return eCollision_Environment;
                    }

                    break;
                }

                // ONE STEP MOVED
                case eMovedOneStep:

                    // check path (todo: maybe this step could be skipped, depending on the workspace step sizes)
                    if (!cspace->isPathValid(lastExtendNode->configuration, tmpConfig))
                    {
                        if (verbose)
                        {
                            cout << __FUNCTION__ << ": aborting moveArmToGraspPos, checkPath fails, todo: check GRASP of last valid config.. " << endl;
                        }

                        return eCollision_Environment;
                    }

                    // ADD IT TO RRT TREE
                    r = tree->appendPath(lastExtendNode, tmpConfig, &lastAddedID);

                    if (!r)
                    {
                        cout << __FUNCTION__ << ": aborting moveArmToGraspPos, appendPath failed?! " << endl;
                        return eError;
                    }

                    processNodes(lastExtendNode->ID, lastAddedID);

                    // update "last" data
                    lastExtendNode = tree->getNode((unsigned int)lastAddedID);

                    // mark last extended node (shouldn't be selected in any future nearest neighbor searches)
                    // 2 -> this node was created during a moveArmToGraspPosition action
                    lastExtendNode->status = 2;

                    lastDist = dist;
                    dist =  VirtualRobot::MathTools::getCartesianPoseDiff(mapConfigTcp[lastExtendNode], targetPose);

                    if ((dist - lastDist) > 2.0f * cartSamplingPosStepSize)
                    {
                        if (verbose)
                        {
                            cout << __FUNCTION__ << ": Stop no dist improvement: last: " << lastDist << ", dist: " << dist << endl;
                        }

                        return eTrapped;
                    }

                    loopCount++;
                    break;

                case eJointBoundaryViolation:
                    if (verbose)
                    {
                        cout << __FUNCTION__ << " Stopping: Joint limits reached " << endl;
                    }

                    return eTrapped;
                    break;

                default:
                    cout << __FUNCTION__ << ": result nyi " << res << endl;
                    break;
            }

            if (dist <= fMaxDist_GraspGoalReached)
            {
                if (verbose)
                {
                    cout << __FUNCTION__ << " grasp goal position reached, dist: " << dist << endl;
                }

                // check grasp score of last valid config
                float fGraspScore = calculateGraspScore(lastExtendNode->configuration, lastExtendNode->ID, true);

                if (fGraspScore >= graspQualityMinScore)
                {
                    cout << __FUNCTION__ << ": found valid grasp (score:" << fGraspScore << ")" << endl;
                    return eGoalReached;
                }
                else if (fGraspScore > 0.0f)
                {
                    cout << __FUNCTION__ << ": found valid grasp (score:" << fGraspScore << ")" << endl;
                    return eGraspablePoseReached;
                }
                else
                {
                    if (verbose)
                    {
                        cout << __FUNCTION__ << ": aborting moveArmToGraspPos, goal pos reached but zero grasp score " << endl;
                    }

                    return eTrapped;
                }
            }
        }

        if (verbose)
        {
            cout << __FUNCTION__ << " max lops reached, dist: " << dist << endl;
        }

        return eTrapped;
    }

    void GraspRrt::limitWorkspaceStep(Eigen::Matrix4f& p)
    {
        float distPos = p.block(0, 3, 3, 1).norm();
        Eigen::Vector3f axis;
        float angle;
        //MathHelpers::quat2AxisAngle(&(pPosQuat[3]),axis,&angle);
        VirtualRobot::MathTools::eigen4f2axisangle(p, axis, angle);
        //cout << "Delta in Workspace: pos:" << deltaX << "," << deltaY << "," << deltaZ << " Dist: " << distPos << endl;
        //cout << "Delta in Workspace: ori:" << angle << endl;
        float distOri = fabs(angle);

        float factorPos = 1.0f;
        float factorOri = 1.0f;

        if (distPos > cartSamplingPosStepSize || distOri > cartSamplingOriStepSize)
        {
            if (distPos > cartSamplingPosStepSize)
            {
                factorPos = cartSamplingPosStepSize / distPos;
            }

            if (distOri > cartSamplingOriStepSize)
            {
                factorOri = cartSamplingOriStepSize / distOri;
            }

            if (factorPos < factorOri)
            {
                factorOri = factorPos;
            }

            if (factorOri < factorPos)
            {
                factorPos = factorOri;
            }
        }

        /*pPosQuat[0] *= factorPos;
        pPosQuat[1] *= factorPos;
        pPosQuat[2] *= factorPos;
        MathHelpers::axisAngle2Quat(axis,angle*factorOri,&(pPosQuat[3]));*/

        Eigen::Matrix4f m = VirtualRobot::MathTools::axisangle2eigen4f(axis, angle * factorOri);
        m.block(0, 3, 3, 1) = p.block(0, 3, 3, 1) * factorPos;
        p = m;
    }


    GraspRrt::MoveArmResult GraspRrt::createWorkSpaceSamplingStep(const Eigen::Matrix4f& currentPose, const Eigen::Matrix4f& goalPose, Eigen::VectorXf& storeCSpaceConf)
    {
        Eigen::Matrix4f deltaPose = goalPose * currentPose.inverse();

        // we need the translational error in global coords
        deltaPose.block(0, 3, 3, 1) = goalPose.block(0, 3, 3, 1) - currentPose.block(0, 3, 3, 1);

        /*
        cout << "Current pose:" << endl;
        cout << currentPose << endl;;
        cout << "goal pose:" << endl;
        cout << goalPose << endl;;
        cout << "delta pose:" << endl;
        cout << deltaPose << endl;;
        */

        limitWorkspaceStep(deltaPose);
        //MathTools::PosQuat2PosRPY(pDeltaPosQuat_Global,pDeltaPosRPY_Global);

        /*
        cout << "delta pose (limit):" << endl;
        cout << deltaPose << endl;;
        cout << "---------------" << endl;
        */

        return moveArmDiffKin(deltaPose, storeCSpaceConf);
    }
    GraspRrt::MoveArmResult GraspRrt::createWorkSpaceSamplingStep(const Eigen::Matrix4f& goalPose, CSpaceNodePtr extendNode, Eigen::VectorXf& storeCSpaceConf)
    {
        // calculate delta
        /*float pDeltaPosRPY_Global[6];
        float pDeltaPosQuat_Global[7];
        for (int i=0;i<3;i++)
        {
            pDeltaPosQuat_Global[i] = pCartGoalPose[i] - pExtendNode->cartPosTCP1[i];
        }
        MathHelpers::deltaQuat(&(pExtendNode->cartPosTCP1[3]),&(pCartGoalPose[3]),&(pDeltaPosQuat_Global[3]));
        */
        robot->setJointValues(rns, extendNode->configuration);
        Eigen::Matrix4f currentPose = mapConfigTcp[extendNode];
        return createWorkSpaceSamplingStep(currentPose, goalPose, storeCSpaceConf);
    }

    GraspRrt::MoveArmResult GraspRrt::moveArmDiffKin(const Eigen::Matrix4f& deltaPose, Eigen::VectorXf& storeCSpaceConf)
    {
        Eigen::VectorXf startConfig(rns->getSize());
        rns->getJointValues(startConfig);

        // get InvJac
        Eigen::VectorXf e(6);

        // The translational error is just the vector  between the actual and the target position
        e.segment(0, 3) = deltaPose.block(0, 3, 3, 1);

        Eigen::AngleAxis<float> aa(deltaPose.block<3, 3>(0, 0));
        e.segment(3, 3) = aa.axis() * aa.angle();

        // Calculate the IK
        Eigen::VectorXf dTheta = diffIK->getPseudoInverseJacobianMatrix(eef->getGCP()) * e;



        /*std::string sKinChain = m_pCSpace->getNameOfKinematicChain();
        CKinematicChain* pKinChain = m_pRobot->GetKinematicChain(sKinChain);
        if (!pKinChain)
        {
            std::cout << "GraspRrt::plan:Error: planner: no kinematic chain with name " << sKinChain << std::endl;
            return eFatalError;
        }
        int nrOfJoints = pKinChain->GetNoOfNodes();

        Matrix invJac;
        //invJac = m_pRobot->getInverseJacobian(pKinChain);
        invJac = m_pEfficientJacobain->getInverseJacobain();

        ColumnVector delatGlobal(6);
        delatGlobal << pDeltaPosRPY_Global[0] << pDeltaPosRPY_Global[1] << pDeltaPosRPY_Global[2] << pDeltaPosRPY_Global[3] << pDeltaPosRPY_Global[4] << pDeltaPosRPY_Global[5];
        std::vector<CRobotNode*> nodes;
        m_pRobot->CollectKinematicChainNodes(nodes, sKinChain);
        int nNrOfNodes = (int)nodes.size();

        ColumnVector storeJointDelta(nNrOfNodes);
        storeJointDelta = invJac*delatGlobal;

        */

        storeCSpaceConf.resize(cspace->getDimension());

        // apply values
        for (int i = 0; i < (int)cspace->getDimension(); i++)
        {
            storeCSpaceConf[i] = startConfig[i] + dTheta[i];
        }

        bool r1 = cspace->isInBoundary(storeCSpaceConf);

        if (!r1)
        {
            return eJointBoundaryViolation;
        }

        // TODO: maybe the planner could speed up by testing TCP / arm collisions
        // this collision check does not distinguish between TCP or arm and grasp object or environment collisions
        bool r2 = cspace->isCollisionFree(storeCSpaceConf);

        if (!r2)
        {
            return eCollision_Environment;
        }

        // apply values
        /*float pV = new float[nNrOfNodes];

        for (int i = 0; i<nNrOfNodes; i++)
            pV[i] = nodes[i]->GetJointValue()+storeJointDelta(i+1);
        m_pRobot->SetJointValues(pV,sKinChain);

        delete []pV;*/

        return eMovedOneStep;

    }


    float GraspRrt::calculateGraspScore(const Eigen::VectorXf& c, int nId, bool bStoreGraspInfoOnSuccess)
    {
        SABA_ASSERT(graspQualityMeasure);
        clock_t timeStart = clock();
        performanceMeasure.numberOfGraspScorings++;

        robot->setJointValues(rns, c);
        VirtualRobot::EndEffector::ContactInfoVector contactsAll = eef->closeActors(graspCollisionObjects);
        VirtualRobot::EndEffector::ContactInfoVector contacts;

        // we only need the targetObject contacts
        for (size_t i = 0; i < contactsAll.size(); i++)
        {
            if (contactsAll[i].obstacle == targetObject)
            {
                contacts.push_back(contactsAll[i]);
            }
        }

        eef->openActors();

		if ((int)contacts.size() < minGraspContacts)
        {
            if (verbose)
            {
                cout << __FUNCTION__ << ": Low number of contacts -> Zero Grasp Score " << endl;
                cout << "Fingers: " ;

                for (int i = 0; i < (int)contacts.size(); i++)
                {
                    cout << contacts[i].actor->getName() << ", ";
                }

                cout << endl;
            }

            clock_t timeEnd = clock();
            performanceMeasure.scoreGraspTimeMS += ((float)(timeEnd - timeStart) / (float)CLOCKS_PER_SEC) * 1000.0f;
            return 0.0f;
        }


        float fScore = 0.0f;
        float fScoreContacts = 0.0f;
        // get distance to target
        Eigen::Vector3f P1, P2;
        int nId1, nId2;
        float fDistTarget = targetObject->getCollisionChecker()->calculateDistance(targetObject->getCollisionModel(), gcpOject->getCollisionModel(), P1, P2, &nId1, &nId2);


        graspQualityMeasure->setContactPoints(contacts);
        graspQualityMeasure->calculateGraspQuality();
        fScore = graspQualityMeasure->getGraspQuality();

        bool isFC = graspQualityMeasure->isValid();
        cout << __FUNCTION__ << ": Grasp Measure Score:" << fScore << endl;
        cout << __FUNCTION__ << ": IsGraspForceClosure/isValid:" << isFC << endl;
        cout << __FUNCTION__ << ": Nr of Contacts Score:" << fScoreContacts << endl;

        if (!isFC)
        {
            fScore = 0.0f;
        }

        //else
        //  fScore = 1.0f;

        if (bStoreGraspInfoOnSuccess && fScore > 0.0f)
        {
            GraspInfo info;
            info.distanceToObject = (float)fDistTarget;
            info.graspScore = fScore;
            info.rrtNodeId = nId;
            //info.contacts = (int)contacts.size();
            Eigen::Matrix4f mMat;

            // we store the grasps relatively to the tcp instead the gcp, since this is expected this way in Simox
            //mMat = eef->getGCP()->toLocalCoordinateSystem(targetObject->getGlobalPose());
            mMat = eef->getTcp()->toLocalCoordinateSystem(targetObject->getGlobalPose());
            //CFeasibleGrasp::ComputeObjectPoseInHandFrame(m_pManipulationObject,m_pEndEffector,mMat);
            info.handToObjectTransform = mMat;
            info.contacts = contacts;
            graspInfoMutex.lock();
            grasps.push_back(info);
            graspInfoMutex.unlock();
        }

        clock_t timeEnd = clock();
        performanceMeasure.scoreGraspTimeMS += ((float)(timeEnd - timeStart) / (float)CLOCKS_PER_SEC) * 1000.0f;

        return fScore;
    }

    /*
    // todo: this should go to the demo
    void GraspRrt::initGraspMeasurement()
    {
        if (!m_pManipulationObject || !m_pGraspQualityMeasure)
            return;
        SbVec3f VecCOM;
        SoSeparator* pObjSep = m_pManipulationObject->GetIVModel();
        m_pManipulationObject->getCenterOfMass(VecCOM);
        cout << "Center of Mass " << VecCOM[0] << "," << VecCOM[1] << "," << VecCOM[2] << endl;
        SoGetBoundingBoxAction *objectBBAction = new SoGetBoundingBoxAction(SbViewportRegion(0, 0));
        SbBox3f objectBB;
        if (pObjSep==NULL)
        {
            printf ("pObject: NULL Data..\n");
            return;
        }
        pObjSep->getBoundingBox(objectBBAction);
        objectBB = objectBBAction->getBoundingBox();
        SbVec3f objectBBSize;
        objectBB.getSize(objectBBSize[0],objectBBSize[1],objectBBSize[2]);
        double objectLength = max(max(objectBBSize[0],objectBBSize[1]),objectBBSize[2]);
        GraspStudio::Vec3D objectCoM;
        objectCoM.x = VecCOM[0];
        objectCoM.y = VecCOM[1];
        objectCoM.z = VecCOM[2];
        m_pGraspQualityMeasure->SetObjectProperties(objectCoM, objectLength, (SoNode*)pObjSep);
        m_pGraspQualityMeasure->CalculateObjectProperties();
    }*/

    void GraspRrt::printGraspInfo(GraspInfo& GrInfo)
    {
        cout << "Grasp Info:" << endl;
        cout << "   Distance to object: " << GrInfo.distanceToObject << endl;
        cout << "   Grasp score: " << GrInfo.graspScore << endl;
        cout << "   RRT Node ID: " << GrInfo.rrtNodeId << endl;
        cout << "   Contacts stored: " << GrInfo.contacts.size() << endl;
    }


    void GraspRrt::printConfig(bool printOnlyParams)
    {
        if (!printOnlyParams)
        {
            std::cout << "-- GraspRrt config --" << std::endl;
            std::cout << "------------------------------" << std::endl;
        }

        if (cspace && cspace->getRobot())
        {
            std::cout << " Robot: " << cspace->getRobot()->getName() << std::endl;
        }

        if (eef)
        {
            std::cout << "   EndEffector: " << eef->getName() << std::endl;

            if (eef->getGCP())
            {
                std::cout << "   GCP: " << eef->getGCP()->getName() << std::endl;
            }
        }

        if (rns)
        {
            std::cout << "   RNS: " << rns->getName() << std::endl;
        }

        cout << "   Cart Step Size: " << cartSamplingPosStepSize << ", Orientational step size: " << cartSamplingPosStepSize << endl;
        Rrt::printConfig(true);

        if (!printOnlyParams)
        {
            std::cout << "------------------------------" << std::endl;
        }
    }

    ApproachDiscretizationPtr GraspRrt::getPoseRelationSphere()
    {
        return poseSphere;
    }

    void GraspRrt::getGraspInfoResult(GraspRrt::GraspInfoVector& storeGraspInfo)
    {
        graspInfoMutex.lock();
        storeGraspInfo = grasps;
        graspInfoMutex.unlock();
    }

    bool GraspRrt::getGraspInfoResult(int nIndex, GraspInfo& storeGraspInfo)
    {
        graspInfoMutex.lock();

        if (nIndex < 0 || nIndex >= (int)grasps.size())
        {
            cout << __FUNCTION__ << ": index out of range: " << nIndex << endl;
            graspInfoMutex.unlock();
            return false;
        }

        storeGraspInfo = grasps[nIndex];
        graspInfoMutex.unlock();
        return true;
    }

    int GraspRrt::getNrOfGraspInfoResults()
    {
        int nRes;
        graspInfoMutex.lock();
        nRes = (int)grasps.size();
        graspInfoMutex.unlock();
        return nRes;
    }

    GraspRrt::PlanningPerformance GraspRrt::getPerformanceMeasurement()
    {
        return performanceMeasure;
    }


    bool GraspRrt::processNodes(unsigned int startId, unsigned int endId)
    {
        if (!tree)
        {
            return false;
        }

        int nActId = endId;
        CSpaceNodePtr currentNode;

        while (nActId >= 0 && nActId != startId)
        {
            currentNode = tree->getNode(nActId);
            processNode(currentNode);

            nActId = currentNode->parentID;
        }

        return true;
    }

    bool GraspRrt::processNode(CSpaceNodePtr n)
    {
        if (!n)
        {
            return false;
        }

        // get tcp pose
        robot->setJointValues(rns, n->configuration);

        // using gcp!
        Eigen::Matrix4f p = eef->getGCP()->getGlobalPose();
        mapConfigTcp[n] = p;
        Eigen::Vector3f dist = p.block(0, 3, 3, 1) - targetObjectPosition;

        if (poseSphere && dist.squaredNorm() < tryGraspsDistance2)
        {
            poseSphere->addCSpaceNode(p.block(0, 3, 3, 1), n);
        }

        return true;

    }

	void GraspRrt::setMinGraspContacts(int nr)
	{
		minGraspContacts = nr;
	}
}
