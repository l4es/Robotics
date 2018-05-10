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
#ifndef _VirtualRobot_CollisionCheckerImplementation_h_
#define _VirtualRobot_CollisionCheckerImplementation_h_

#include "../VirtualRobotImportExport.h"

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace VirtualRobot
{
    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionCheckerImplementation
    {
    public:
        friend class CollisionChecker;

        CollisionCheckerImplementation()
        {
            automaticSizeCheck = true;
            debugOutput = false;
        }
        virtual ~CollisionCheckerImplementation() {}

        virtual float calculateDistance(CollisionModelPtr model1, CollisionModelPtr model2, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int* trID1 = NULL, int* trID2 = NULL) = 0;
        virtual bool checkCollision(CollisionModelPtr model1, CollisionModelPtr model2) = 0; //, Eigen::Vector3f *storeContact = NULL) = 0;

        virtual void setAutomaticSizeCheck(bool checkSizeOnColModelCreation)
        {
            automaticSizeCheck = checkSizeOnColModelCreation;
        }

        virtual void enableDebugOutput(bool e)
        {
            debugOutput = e;
        }

        bool debugOutput;

    protected:
        bool automaticSizeCheck;
    };

} // namespace

#endif
