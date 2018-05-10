
#include "CollisionCheckerDummy.h"
#include "CollisionModelDummy.h"
#include "../../CollisionChecker.h"
#include "../../CollisionModel.h"

namespace VirtualRobot
{

    //----------------------------------------------------------------------
    // class CollisionChecker constructor
    //----------------------------------------------------------------------
    CollisionCheckerDummy::CollisionCheckerDummy(): CollisionCheckerImplementation()
    {
    }

    //----------------------------------------------------------------------
    // class CollisionChecker destructor
    //----------------------------------------------------------------------
    CollisionCheckerDummy::~CollisionCheckerDummy()
    {
    }


    float CollisionCheckerDummy::calculateDistance(CollisionModelPtr model1, CollisionModelPtr model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1, int* trID2)
    {
        return -1.0f;
    }

    bool CollisionCheckerDummy::checkCollision(CollisionModelPtr model1, CollisionModelPtr model2)
    {
        return false;
    }

} // namespace


