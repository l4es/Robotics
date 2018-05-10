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
#ifndef _VirtualRobot_CollisionModelImplementation_h_
#define _VirtualRobot_CollisionModelImplementation_h_

#include "../VirtualRobotImportExport.h"

#include <string>
#include <vector>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace VirtualRobot
{

    class CollisionChecker;
    class CollisionModel;

    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionModelImplementation
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        friend class CollisionModel;

        /*!Standard Constructor
        If collision checks should be done in parallel, different CollisionCheckers can be specified.
        */
        CollisionModelImplementation(TriMeshModelPtr modelData, CollisionCheckerPtr pColChecker, int id)
        {
            this->modelData = modelData;
            this->id = id;
        };

        /*!Standard Destructor
        */
        virtual ~CollisionModelImplementation() {};


        /*!
        Sets the position of the internal colModel data structure.
        */
        void setGlobalPose(const Eigen::Matrix4f& m)
        {
            globalPose = m;
        }
        inline const Eigen::Matrix4f& getGlobalPose() const
        {
            return globalPose;
        }


        virtual void print()
        {
            cout << "Dummy Collision Model Implementation..." << endl;
        };

        TriMeshModelPtr getTriMeshModel()
        {
            return modelData;
        }

    protected:

        //! delete all data
        virtual void destroyData() = 0;

        TriMeshModelPtr modelData;

        int id;

        Eigen::Matrix4f globalPose;
    };



} // namespace

#endif
