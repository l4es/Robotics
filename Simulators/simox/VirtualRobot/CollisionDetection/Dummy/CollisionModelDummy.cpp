
#include "CollisionModelDummy.h"

#include "../../CollisionChecker.h"
#include "CollisionCheckerDummy.h"
#include <algorithm>

namespace VirtualRobot
{

    CollisionModelDummy::CollisionModelDummy(CollisionCheckerPtr pColChecker)
        : CollisionModelImplementation(pColChecker)
    {
        if (!pColChecker)
        {
            cout << "CollisionModelDummy: warning, no col checker..." << endl;
        }

        VR_WARNING << ": THIS IS A DUMMY IMPLEMENTATION WITHOUT ANY FUNCTIONALITY..." << endl;
    }


    CollisionModelDummy::~CollisionModelDummy()
    {
        destroyData();
    }


    void CollisionModelDummy::destroyData()
    {
    }

    /*
    void CollisionModelDummy::setGlobalPose (const Eigen::Matrix4f &m)
    {

    }*/

} // namespace
