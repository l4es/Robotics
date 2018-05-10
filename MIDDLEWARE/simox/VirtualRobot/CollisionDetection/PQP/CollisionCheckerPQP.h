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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CollisionCheckerPQP_h_
#define _VirtualRobot_CollisionCheckerPQP_h_

#include "../../VirtualRobotImportExport.h"

#include "../CollisionCheckerImplementation.h"


#include <string>
#include <vector>

#include "PQP++/PQP_Compile.h"
#include "PQP++/PQP.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace VirtualRobot
{
    /*!
        This implementation encapsulates the PQP collision checker.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionCheckerPQP : public CollisionCheckerImplementation
    {
    public:
        friend class CollisionChecker;

        CollisionCheckerPQP();
        virtual ~CollisionCheckerPQP();

        virtual float calculateDistance(CollisionModelPtr model1, CollisionModelPtr model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1 = NULL, int* trID2 = NULL);
        virtual bool checkCollision(CollisionModelPtr model1, CollisionModelPtr model2); //, Eigen::Vector3f *storeContact = NULL);

        /*!
        If continuous collision detection (CCD) is supported, this method can be used to detect collisions on the path
        from the current pose of the collision models to the goal poses.
        true -> collision (then the time of contact [0..1] is stored to fStoreTOC)
        */
        //bool CheckContinuousCollision (CollisionModel *model1, Eigen::Matrix4f &mGoalPose1, CollisionModel *model2, Eigen::Matrix4f &mGoalPose2, float &fStoreTOC);


        float getMinDistance(boost::shared_ptr<PQP::PQP_Model> m1, boost::shared_ptr<PQP::PQP_Model> m2, const Eigen::Matrix4f& mat1, const Eigen::Matrix4f& mat2);
        float getMinDistance(boost::shared_ptr<PQP::PQP_Model> m1, boost::shared_ptr<PQP::PQP_Model> m2, const Eigen::Matrix4f& mat1, const Eigen::Matrix4f& mat2, Eigen::Vector3f& storeP1, Eigen::Vector3f& storeP2, int* storeID1, int* storeID2);

        void GetPQPDistance(const boost::shared_ptr<PQP::PQP_Model>& model1, const boost::shared_ptr<PQP::PQP_Model>& model2, const Eigen::Matrix4f& matrix1, const Eigen::Matrix4f& matrix2, PQP::PQP_DistanceResult& pqpResult);


        /*!
        Does the underlying collision detection library support discrete collision detection.
        */
        static bool IsSupported_CollisionDetection()
        {
            return true;
        }

        /*!
        Does the underlying collision detection library support continuous collision detection.
        */
        static bool IsSupported_ContinuousCollisionDetection()
        {
            return false;
        }

        /*!
        Does the underlying collision detection library support distance calculations.
        */
        static bool IsSupported_DistanceCalculations()
        {
            return true;
        }

        /*!
        Does the underlying collision detection library support threadsafe access.
        E.g. multiple threads query the collision checker asynchronously.
        */
        static bool IsSupported_Multithreading_Threadsafe()
        {
            return false;
        }

        /*!
        Does the underlying collision detection library support multiple instances of the collision checker.
        E.g. one per thread.
        */
        static bool IsSupported_Multithreading_MultipleColCheckers()
        {
            return true;
        }

    protected:
        PQP::PQP_Checker* pqpChecker;
    };

} // namespace


#endif
