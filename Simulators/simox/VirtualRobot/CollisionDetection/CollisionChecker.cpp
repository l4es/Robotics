
#include "CollisionChecker.h"
#include "CollisionModel.h"
#include "../SceneObjectSet.h"
#include "../SceneObject.h"
#include "../Robot.h"
#include "../VirtualRobotException.h"

#include <cfloat>

#include <Eigen/Core>
#include <Eigen/Geometry>


#if defined(VR_COLLISION_DETECTION_PQP)
#define COL_CHECKER_IMPL CollisionCheckerPQP
#else
#define COL_CHECKER_IMPL CollisionCheckerDummy
#endif




namespace VirtualRobot
{

    namespace
    {
        boost::mutex mutex;
    };

    CollisionCheckerPtr CollisionChecker::globalCollisionChecker;

    CollisionChecker::Cleanup::~Cleanup()
    {
        boost::lock_guard<boost::mutex> lock(mutex);
        CollisionChecker::globalCollisionChecker.reset();
    }


    CollisionCheckerPtr CollisionChecker::getGlobalCollisionChecker()
    {
        static Cleanup _Cleanup;

        if (true)
        {
            boost::lock_guard<boost::mutex> lock(mutex);

            if (!globalCollisionChecker)
            {
                globalCollisionChecker.reset(new CollisionChecker());
            }
        }

        return globalCollisionChecker;
    }

    //----------------------------------------------------------------------
    // class CollisionChecker constructor
    //----------------------------------------------------------------------
    CollisionChecker::CollisionChecker()
    {
        initialized = true;
        debugOutput = false; // ENABLES OUTPUT OF COLLISION MODEL TRIANGLES
        automaticSizeCheck = true;

        collisionCheckerImplementation.reset(new COL_CHECKER_IMPL());
    }

    //----------------------------------------------------------------------
    // class CollisionChecker destructor
    //----------------------------------------------------------------------
    CollisionChecker::~CollisionChecker()
    {
    }


    float CollisionChecker::calculateDistance(SceneObjectSetPtr model1, SceneObjectSetPtr model2)
    {
        VR_ASSERT(model1 && model2);
        return calculateDistance(model1, model2, tmpV1, tmpV2, NULL, NULL);
    }

    float CollisionChecker::calculateDistance(CollisionModelPtr model1, SceneObjectSetPtr model2)
    {
        VR_ASSERT(model1 && model2);
        return calculateDistance(model1, model2, tmpV1, tmpV2, NULL, NULL);
    }

    float CollisionChecker::calculateDistance(CollisionModelPtr model1, CollisionModelPtr model2)
    {
        VR_ASSERT(model1 && model2);
        return calculateDistance(model1, model2, tmpV1, tmpV2, NULL, NULL);
    }


    float CollisionChecker::calculateDistance(SceneObjectPtr model1, SceneObjectSetPtr model2)
    {
        VR_ASSERT(model1 && model2);
		RobotPtr r = boost::dynamic_pointer_cast<Robot>(model1);
		if (r)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			return calculateDistance(robotModels, model2);
		}
		else
			return calculateDistance(model1->getCollisionModel(), model2, tmpV1, tmpV2, NULL, NULL);
    }

    float CollisionChecker::calculateDistance(SceneObjectPtr model1, SceneObjectPtr model2)
    {
        VR_ASSERT(model1 && model2);
		RobotPtr r = boost::dynamic_pointer_cast<Robot>(model1);
		RobotPtr r2 = boost::dynamic_pointer_cast<Robot>(model2);
		if (r && r2)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			SceneObjectSetPtr robotModels2 = getRobotModels(r2);
			return calculateDistance(robotModels, robotModels2);
		}
		else if (r && !r2)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			return calculateDistance(model2, robotModels);
		}
		else if (!r && r2)
		{
			SceneObjectSetPtr robotModels2 = getRobotModels(r2);
			return calculateDistance(model1, robotModels2);
		}
		else
			return calculateDistance(model1->getCollisionModel(), model2->getCollisionModel(), tmpV1, tmpV2, NULL, NULL);
    }


    float CollisionChecker::calculateDistance(SceneObjectPtr model1, SceneObjectSetPtr model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        VR_ASSERT(model1 && model2);
		RobotPtr r = boost::dynamic_pointer_cast<Robot>(model1);
		if (r)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			return calculateDistance(robotModels, model2, P1, P2, trID1, trID2);
		}
		else
			return calculateDistance(model1->getCollisionModel(), model2, P1, P2, trID1, trID2);
    }

    float CollisionChecker::calculateDistance(SceneObjectPtr model1, SceneObjectPtr model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        VR_ASSERT(model1 && model2);
		RobotPtr r = boost::dynamic_pointer_cast<Robot>(model1);
		RobotPtr r2 = boost::dynamic_pointer_cast<Robot>(model2);
		if (r && r2)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			SceneObjectSetPtr robotModels2 = getRobotModels(r2);
			return calculateDistance(robotModels, robotModels2, P1, P2, trID1, trID2);
		}
		else if (r && !r2)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			return calculateDistance(model2, robotModels, P2, P1, trID2, trID1);
		}
		else if (!r && r2)
		{
			SceneObjectSetPtr robotModels2 = getRobotModels(r2);
			return calculateDistance(model1, robotModels2, P1, P2, trID1, trID2);
		}
		else
        return calculateDistance(model1->getCollisionModel(), model2->getCollisionModel(), P1, P2, trID1, trID2);
    }

    float CollisionChecker::calculateDistance(SceneObjectSetPtr model1, SceneObjectSetPtr model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        VR_ASSERT(model1 && model2);
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == model2->getCollisionChecker(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == shared_from_this(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT(isInitialized());

        std::vector< CollisionModelPtr > colModels1 = model1->getCollisionModels();
        std::vector< CollisionModelPtr > colModels2 = model2->getCollisionModels();

        if (colModels1.size() == 0 || colModels2.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return -1.0f;
        }

        float fResult = FLT_MAX;
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        int trID1_r;
        int trID2_r;
        std::vector< CollisionModelPtr >::iterator it1 = colModels1.begin();

        while (it1 != colModels1.end())
        {
            std::vector< CollisionModelPtr >::iterator it2 = colModels2.begin();

            while (it2 != colModels2.end())
            {
                float fRes = calculateDistance(*it1, *it2, v1, v2, &trID1_r, &trID2_r);

                if (fRes <= fResult)
                {
                    fResult = fRes;
                    P1 = v1;
                    P2 = v2;

                    if (trID1)
                    {
                        *trID1 = trID1_r;
                    }

                    if (trID2)
                    {
                        *trID2 = trID2_r;
                    }
                }

                it2++;
            }

            it1++;
        }

        return fResult;
    }

    float CollisionChecker::calculateDistance(CollisionModelPtr model1, SceneObjectSetPtr model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        VR_ASSERT(model1 && model2);
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == model2->getCollisionChecker(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == shared_from_this(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT(isInitialized());

        std::vector< CollisionModelPtr > colModels = model2->getCollisionModels();

        if (colModels.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return -1.0f;
        }

        float fResult = FLT_MAX;
        Eigen::Vector3f v1;
        Eigen::Vector3f v2;
        int trID1_r;
        int trID2_r;
        std::vector< CollisionModelPtr >::iterator it = colModels.begin();

        while (it != colModels.end())
        {
            float fRes = calculateDistance(model1, *it, v1, v2, &trID1_r, &trID2_r);

            if (fRes <= fResult)
            {
                fResult = fRes;
                P1 = v1;
                P2 = v2;

                if (trID1)
                {
                    *trID1 = trID1_r;
                }

                if (trID2)
                {
                    *trID2 = trID2_r;
                }
            }

            it++;
        }

        return fResult;
    }

    float CollisionChecker::calculateDistance(CollisionModelPtr model1, CollisionModelPtr model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        VR_ASSERT(model1 && model2);
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == model2->getCollisionChecker(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == shared_from_this(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT(isInitialized());

        return collisionCheckerImplementation->calculateDistance(model1, model2, P1, P2, trID1, trID2);
    }



    bool CollisionChecker::checkCollision(std::vector<CollisionModelPtr>& model1, CollisionModelPtr model2)
    {
        VR_ASSERT(model2);
        VR_ASSERT(isInitialized());

        if (model1.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return false;
        }

        std::vector< CollisionModelPtr >::iterator it1 = model1.begin();

        while (it1 != model1.end())
        {
            if (checkCollision(*it1, model2))
            {
                return true;
            }

            it1++;
        }

        return false;
    }

    bool CollisionChecker::checkCollision(SceneObjectSetPtr model1, SceneObjectSetPtr model2)
    {
        VR_ASSERT(model1 && model2);
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == model2->getCollisionChecker(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == shared_from_this(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT(isInitialized());

        std::vector< CollisionModelPtr > vColModels1 = model1->getCollisionModels();
        std::vector< CollisionModelPtr > vColModels2 = model2->getCollisionModels();

        if (vColModels1.size() == 0 || vColModels2.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return false;
        }

        std::vector< CollisionModelPtr >::iterator it1 = vColModels1.begin();

        while (it1 != vColModels1.end())
        {
            std::vector< CollisionModelPtr >::iterator it2 = vColModels2.begin();

            while (it2 != vColModels2.end())
            {
                if (checkCollision(*it1, *it2))
                {
                    return true;
                }

                it2++;
            }

            it1++;
        }

        return false;
    }

    bool CollisionChecker::checkCollision(SceneObjectPtr model1, SceneObjectSetPtr model2)
    {
        VR_ASSERT(model1 && model2);
		RobotPtr r = boost::dynamic_pointer_cast<Robot>(model1);
		if (r)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			return checkCollision(robotModels, model2);
		} else
			return checkCollision(model1->getCollisionModel(), model2);
    }

	SceneObjectSetPtr CollisionChecker::getRobotModels(RobotPtr r)
	{
		SceneObjectSetPtr result(new SceneObjectSet(r->getName(), shared_from_this()));
		std::vector<RobotNodePtr> cm = r->getRobotNodes();
		for (size_t i = 0; i < cm.size(); i++)
		{
			if (cm[i]->getCollisionModel())
				result->addSceneObject(cm[i]);
		}
		return result;
	}


    bool CollisionChecker::checkCollision(CollisionModelPtr model1, SceneObjectSetPtr model2)
    {
        VR_ASSERT(model1 && model2);
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == model2->getCollisionChecker(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == shared_from_this(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT(isInitialized());

        std::vector< CollisionModelPtr > vColModels = model2->getCollisionModels();

        if (vColModels.size() == 0)
        {
            VR_WARNING << "no internal data..." << endl;
            return false;
        }

        std::vector< CollisionModelPtr >::iterator it = vColModels.begin();

        while (it != vColModels.end())
        {
            if (checkCollision(model1, *it))
            {
                return true;
            }

            it++;
        }

        return false;
    }

    bool CollisionChecker::checkCollision(SceneObjectPtr model1, SceneObjectPtr model2)
    {
        VR_ASSERT(model1 && model2);
		RobotPtr r = boost::dynamic_pointer_cast<Robot>(model1);
		RobotPtr r2 = boost::dynamic_pointer_cast<Robot>(model2);
		if (r && r2)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			SceneObjectSetPtr robotModels2 = getRobotModels(r2);
			return checkCollision(robotModels, robotModels2);
		}
		else if (r && !r2)
		{
			SceneObjectSetPtr robotModels = getRobotModels(r);
			return checkCollision(model2, robotModels);
		}
		else if (!r && r2)
		{
			SceneObjectSetPtr robotModels2 = getRobotModels(r2);
			return checkCollision(model1, robotModels2);
		}
		else
			return checkCollision(model1->getCollisionModel(), model2->getCollisionModel());
    }

    bool CollisionChecker::checkCollision(CollisionModelPtr model1, CollisionModelPtr model2)
    {
        VR_ASSERT(model1 && model2);
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == model2->getCollisionChecker(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT_MESSAGE(model1->getCollisionChecker() == shared_from_this(), "Collision models are linked to different Collision Checker instances");
        VR_ASSERT(isInitialized());

        return collisionCheckerImplementation->checkCollision(model1, model2);//, storeContact);
    }

    /*
    bool CollisionChecker::getAllCollisonTriangles (SceneObjectSetPtr model1, SceneObjectSetPtr model2, std::vector<int> &storePairs)
    {
        if (!model1 || !model2)
        {
            printf ("CollisionChecker:GetAllCollisongTriangles - NULL data...\n");
            return false;
        }
        if (model1->getCollisionChecker() != model2->getCollisionChecker() || model1->getCollisionChecker()!=shared_from_this())
        {
            VR_WARNING << "Could not go on, collision models are linked to different Collision Checker instances." << endl;
            return false;
        }
        if (!isInitialized())
        {
            VR_WARNING << "not initialized." << endl;
            return false;
        }
        return collisionCheckerImplementation->getAllCollisonTriangles(model1,model2,storePairs);
    }*/


    void CollisionChecker::setAutomaticSizeCheck(bool checkSizeOnColModelCreation)
    {
        automaticSizeCheck = checkSizeOnColModelCreation;
        collisionCheckerImplementation->setAutomaticSizeCheck(automaticSizeCheck);
    }

    /*
    bool CollisionChecker::checkCollision( SbXfBox3f& box1, SbXfBox3f& box2 )
    {
        ////return box1.intersect(box2);

        //box1.setBounds(-100, -100, -100, 100, 100, 100);
        //box2.setBounds(-100, -100, -100, 100, 100, 100);

        //SbMatrix tr;
        //tr.setTranslate(SbVec3f(0,0,0));
        //box1.setTransform(tr);

        //tr.setTranslate(SbVec3f(0, 0, 250));
        //box2.setTransform(tr);

        // OBB Intersection test from Neoengine

        const float fParallellCutoff = 0.99999f;
        bool bParallellAxes = false;

        SbRotation kRotA;
        SbRotation kRotB;
        SbVec3f kTransA;
        SbVec3f kTransB;
        SbMatrix kRotMatA;
        SbMatrix kRotMatB;
        SbVec3f tmpV;
        SbRotation tmpR;

        box1.getTransform().getTransform(kTransA, kRotA, tmpV, tmpR);
        box2.getTransform().getTransform(kTransB, kRotB, tmpV, tmpR);
        kRotMatA.setRotate(kRotA);
        kRotMatB.setRotate(kRotB);

        SbVec3f akAxesA[3] = { SbVec3f(kRotMatA[0]), SbVec3f(kRotMatA[1]), SbVec3f(kRotMatA[2]) };
        SbVec3f akAxesB[3] = { SbVec3f(kRotMatB[0]), SbVec3f(kRotMatB[1]), SbVec3f(kRotMatB[2]) };

        float afDimA[3];
        float afDimB[3];

        float dx, dy, dz;
        box1.getSize(dx, dy, dz);
        afDimA[0] = dx/2;
        afDimA[1] = dy/2;
        afDimA[2] = dz/2;

        box2.getSize(dx, dy, dz);
        afDimB[0] = dx/2;
        afDimB[1] = dy/2;
        afDimB[2] = dz/2;

        //Difference of box positions
        SbVec3f kDiff = kTransB - kTransA;

        ////////Early out test
        //float radiusA2 = 0, radiusB2 = 0;

        //SbBox3f aabb1(box1);
        //SbBox3f aabb2(box2);
        //radiusA2 = (aabb1.getMax()-aabb1.getCenter()).length();
        //radiusB2 = (aabb2.getMax()-aabb2.getCenter()).length();
        //if( ( radiusA2 + radiusB2 ) < kDiff.length() )
        //  return false;

        float afAxisDot[3][3];    // afAxisDot[i][j]    = akAxesA[i]Ptr  akAxesB[j];
        float afAbsAxisDot[3][3]; // afAbsAxisDot[i][j] = |afAxisDot[i][j]|

        float afAxesADotDiff[3];  // afAxesADotDiff[i]  = akAxesA[i]Ptr  kDiff

        //Test all 15 possible separating axes

        //Axis Ax
        //First calculate AxPtr  Bi axis dot products (length of each B axis along Ax)
        afAxisDot[0][0] = akAxesA[0].dot(akAxesB[0]);
        afAxisDot[0][1] = akAxesA[0].dot(akAxesB[1]);
        afAxisDot[0][2] = akAxesA[0].dot(akAxesB[2]);

        //Get absolute value of dot products
        afAbsAxisDot[0][0] = fabsf( afAxisDot[0][0] ); if( afAbsAxisDot[0][0] > fParallellCutoff ) bParallellAxes = true;
        afAbsAxisDot[0][1] = fabsf( afAxisDot[0][1] ); if( afAbsAxisDot[0][1] > fParallellCutoff ) bParallellAxes = true;
        afAbsAxisDot[0][2] = fabsf( afAxisDot[0][2] ); if( afAbsAxisDot[0][2] > fParallellCutoff ) bParallellAxes = true;

        //calculate AxPtr  D dot product (length of center difference along Ax axis)
        afAxesADotDiff[0]  = akAxesA[0].dot(kDiff);

        //  int     iDeepAxis = 0;
        //  SbVec3f kNormal;

        //  float fOverlapMax  = -numeric_limits< float >::max();
        float fOverlap;

        //Check if distance between centers along axis is greater than sum of boxes dimensions along axis
        if( ( fOverlap = fabsf( afAxesADotDiff[0] ) - ( ( afDimA[0] ) + ( afDimB[0]Ptr  afAbsAxisDot[0][0] + afDimB[1]Ptr  afAbsAxisDot[0][1] + afDimB[2]Ptr  afAbsAxisDot[0][2] ) ) ) > 0.0f )
            return false;

        //Axis Ay
        afAxisDot[1][0] = akAxesA[1].dot(akAxesB[0]);
        afAxisDot[1][1] = akAxesA[1].dot(akAxesB[1]);
        afAxisDot[1][2] = akAxesA[1].dot(akAxesB[2]);

        afAbsAxisDot[1][0] = fabsf( afAxisDot[1][0] ); if( afAbsAxisDot[1][0] > fParallellCutoff ) bParallellAxes = true;
        afAbsAxisDot[1][1] = fabsf( afAxisDot[1][1] ); if( afAbsAxisDot[1][1] > fParallellCutoff ) bParallellAxes = true;
        afAbsAxisDot[1][2] = fabsf( afAxisDot[1][2] ); if( afAbsAxisDot[1][2] > fParallellCutoff ) bParallellAxes = true;

        afAxesADotDiff[1]  = akAxesA[1].dot(kDiff);

        if( ( fOverlap = fabsf( afAxesADotDiff[1] ) - ( ( afDimA[1] ) + ( afDimB[0]Ptr  afAbsAxisDot[1][0] + afDimB[1]Ptr  afAbsAxisDot[1][1] + afDimB[2]Ptr  afAbsAxisDot[1][2] ) ) ) > 0.0f )
            return false;

        //Axis Az
        afAxisDot[2][0] = akAxesA[2].dot(akAxesB[0]);
        afAxisDot[2][1] = akAxesA[2].dot(akAxesB[1]);
        afAxisDot[2][2] = akAxesA[2].dot(akAxesB[2]);

        afAbsAxisDot[2][0] = fabsf( afAxisDot[2][0] ); if( afAbsAxisDot[2][0] > fParallellCutoff ) bParallellAxes = true;
        afAbsAxisDot[2][1] = fabsf( afAxisDot[2][1] ); if( afAbsAxisDot[2][1] > fParallellCutoff ) bParallellAxes = true;
        afAbsAxisDot[2][2] = fabsf( afAxisDot[2][2] ); if( afAbsAxisDot[2][2] > fParallellCutoff ) bParallellAxes = true;

        afAxesADotDiff[2]  = akAxesA[2].dot(kDiff);

        if( ( fOverlap = fabsf( afAxesADotDiff[2] ) - ( ( afDimA[2] ) + ( afDimB[0]Ptr  afAbsAxisDot[2][0] + afDimB[1]Ptr  afAbsAxisDot[2][1] + afDimB[2]Ptr  afAbsAxisDot[2][2] ) ) ) > 0.0f )
            return false;

        float fProj;
        //  float fScale;

        //Axis Bx
        //We already have all axis*axis dot products, only calculate center difference along Bx axis and compare
        if( ( fOverlap = fabsf( ( fProj = akAxesB[0].dot(kDiff) ) ) - ( ( afDimA[0]Ptr  afAbsAxisDot[0][0] + afDimA[1]Ptr  afAbsAxisDot[1][0] + afDimA[2]Ptr  afAbsAxisDot[2][0] ) + ( afDimB[0] ) ) ) > 0.0f )
            return false;

        //Axis By
        if( ( fOverlap = fabsf( ( fProj = akAxesB[1].dot(kDiff) ) ) - ( ( afDimA[0]Ptr  afAbsAxisDot[0][1] + afDimA[1]Ptr  afAbsAxisDot[1][1] + afDimA[2]Ptr  afAbsAxisDot[2][1] ) + ( afDimB[1] ) ) ) > 0.0f )
            return false;

        //Axis Bz
        if( ( fOverlap = fabsf( ( fProj = akAxesB[2].dot(kDiff) ) ) - ( ( afDimA[0]Ptr  afAbsAxisDot[0][2] + afDimA[1]Ptr  afAbsAxisDot[1][2] + afDimA[2]Ptr  afAbsAxisDot[2][2] ) + ( afDimB[2] ) ) ) > 0.0f )
            return false;

        //if not contact set, do extra tests to avoid reporting false collisions in parallell axis threshold zone
        if( !bParallellAxes  )
        {

            //Axis Ax X Bx
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[1][0]Ptr  afAxesADotDiff[2] - afAxisDot[2][0]Ptr  afAxesADotDiff[1] ) ) -
                ( ( afDimA[1]Ptr  afAbsAxisDot[2][0] + afDimA[2]Ptr  afAbsAxisDot[1][0] ) + ( afDimB[1]Ptr  afAbsAxisDot[0][2] + afDimB[2]Ptr  afAbsAxisDot[0][1] ) ) ) > 0.0f )
                return false;

            //Axis Ax X By
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[1][1]Ptr  afAxesADotDiff[2] - afAxisDot[2][1]Ptr  afAxesADotDiff[1] ) ) -
                ( ( afDimA[1]Ptr  afAbsAxisDot[2][1] + afDimA[2]Ptr  afAbsAxisDot[1][1] ) + ( afDimB[0]Ptr  afAbsAxisDot[0][2] + afDimB[2]Ptr  afAbsAxisDot[0][0] ) ) ) > 0.0f )
                return false;

            //Axis Ax X Bz
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[1][2]Ptr  afAxesADotDiff[2] - afAxisDot[2][2]Ptr  afAxesADotDiff[1] ) ) -
                ( ( afDimA[1]Ptr  afAbsAxisDot[2][2] + afDimA[2]Ptr  afAbsAxisDot[1][2] ) + ( afDimB[0]Ptr  afAbsAxisDot[0][1] + afDimB[1]Ptr  afAbsAxisDot[0][0] ) ) ) > 0.0f )
                return false;

            //Axis Ay X Bx
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[2][0]Ptr  afAxesADotDiff[0] - afAxisDot[0][0]Ptr  afAxesADotDiff[2] ) ) -
                ( ( afDimA[0]Ptr  afAbsAxisDot[2][0] + afDimA[2]Ptr  afAbsAxisDot[0][0] ) + ( afDimB[1]Ptr  afAbsAxisDot[1][2] + afDimB[2]Ptr  afAbsAxisDot[1][1] ) ) ) > 0.0f )
                return false;

            //Axis Ay X By
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[2][1]Ptr  afAxesADotDiff[0] - afAxisDot[0][1]Ptr  afAxesADotDiff[2] ) ) -
                ( ( afDimA[0]Ptr  afAbsAxisDot[2][1] + afDimA[2]Ptr  afAbsAxisDot[0][1] ) + ( afDimB[0]Ptr  afAbsAxisDot[1][2] + afDimB[2]Ptr  afAbsAxisDot[1][0] ) ) ) > 0.0f )
                return false;

            //Axis Ay X Bz
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[2][2]Ptr  afAxesADotDiff[0] - afAxisDot[0][2]Ptr  afAxesADotDiff[2] ) ) -
                ( ( afDimA[0]Ptr  afAbsAxisDot[2][2] + afDimA[2]Ptr  afAbsAxisDot[0][2] ) + ( afDimB[0]Ptr  afAbsAxisDot[1][1] + afDimB[1]Ptr  afAbsAxisDot[1][0] ) ) ) > 0.0f )
                return false;

            //Axis Az X Bx
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[0][0]Ptr  afAxesADotDiff[1] - afAxisDot[1][0]Ptr  afAxesADotDiff[0] ) ) -
                ( ( afDimA[0]Ptr  afAbsAxisDot[1][0] + afDimA[1]Ptr  afAbsAxisDot[0][0] ) + ( afDimB[1]Ptr  afAbsAxisDot[2][2] + afDimB[2]Ptr  afAbsAxisDot[2][1] ) ) ) > 0.0f )
                return false;

            //Axis Az X By
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[0][1]Ptr  afAxesADotDiff[1] - afAxisDot[1][1]Ptr  afAxesADotDiff[0] ) ) -
                ( ( afDimA[0]Ptr  afAbsAxisDot[1][1] + afDimA[1]Ptr  afAbsAxisDot[0][1] ) + ( afDimB[0]Ptr  afAbsAxisDot[2][2] + afDimB[2]Ptr  afAbsAxisDot[2][0] ) ) ) > 0.0f )
                return false;

            //Axis Az X Bz
            if( ( fOverlap = fabsf( ( fProj = afAxisDot[0][2]Ptr  afAxesADotDiff[1] - afAxisDot[1][2]Ptr  afAxesADotDiff[0] ) ) -
                ( ( afDimA[0]Ptr  afAbsAxisDot[1][2] + afDimA[1]Ptr  afAbsAxisDot[0][2] ) + ( afDimB[0]Ptr  afAbsAxisDot[2][1] + afDimB[1]Ptr  afAbsAxisDot[2][0] ) ) ) > 0.0f )
                return false;
        } // if( bParallellAxes )

        return true;
    }
    */

    bool CollisionChecker::IsSupported_CollisionDetection()
    {
        return COL_CHECKER_IMPL::IsSupported_CollisionDetection();
    }

    bool CollisionChecker::IsSupported_ContinuousCollisionDetection()
    {
        return COL_CHECKER_IMPL::IsSupported_ContinuousCollisionDetection();
    }

    bool CollisionChecker::IsSupported_DistanceCalculations()
    {
        return COL_CHECKER_IMPL::IsSupported_DistanceCalculations();
    }

    bool CollisionChecker::IsSupported_Multithreading_Threadsafe()
    {
        return COL_CHECKER_IMPL::IsSupported_Multithreading_Threadsafe();
    }

    bool CollisionChecker::IsSupported_Multithreading_MultipleColCheckers()
    {
        return COL_CHECKER_IMPL::IsSupported_Multithreading_MultipleColCheckers();
    }

    void CollisionChecker::getContacts(const MathTools::Plane& p, CollisionModelPtr colModel, std::vector< MathTools::ContactPoint >& storeContatcs, float maxDist /*= 1.0f*/)
    {
        THROW_VR_EXCEPTION_IF(!colModel, "NULl data");

        // first check if plane hits bounding box
        BoundingBox bbox = colModel->getBoundingBox(false);
        // enlarge bbox by maxDist
        bbox.min -= Eigen::Vector3f(maxDist, maxDist, maxDist);
        bbox.max += Eigen::Vector3f(maxDist, maxDist, maxDist);

        std::vector <Eigen::Vector3f> ptsBB = bbox.getPoints();

        for (size_t i = 0; i < ptsBB.size(); i++)
        {
            ptsBB[i] = MathTools::transformPosition(ptsBB[i], colModel->getGlobalPose());
        }

        BoundingBox bboxGlobal(ptsBB);

        if (!bboxGlobal.planeGoesThrough(p))
        {
            // plane is not going through bounding box
            return;
        }

        // bbox was hit, test all points...
        std::vector< Eigen::Vector3f > pts = colModel->getModelVeticesGlobal();

        for (std::vector< Eigen::Vector3f >::iterator i = pts.begin(); i != pts.end(); i++)
        {
            if (MathTools::getDistancePointPlane(*i, p) <= maxDist)
            {
                MathTools::ContactPoint contact;
                contact.n = p.n;
                contact.p = *i;
                storeContatcs.push_back(contact);
            }
        }
    }

    /*
    bool CollisionChecker::checkContinuousCollision( CollisionModelPtr model1, SbMatrix &mGoalPose1, CollisionModelPtr model2, SbMatrix &mGoalPose2, float &fStoreTOC )
    {
        return collisionCheckerImplementation->checkContinuousCollision(model1,mGoalPose1,model2,mGoalPose2, fStoreTOC);
    }
    */

} // namespace VirtualRobot

