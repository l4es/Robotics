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
#ifndef _VirtualRobot_GraspSet_h_
#define _VirtualRobot_GraspSet_h_

#include "../VirtualRobotImportExport.h"

#include <string>
#include <vector>

#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include "Grasp.h"
#include <Eigen/Core>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT GraspSet
    {
    public:

        /*!
        */
        GraspSet(const std::string& name, const std::string& robotType, const std::string& eef, const std::vector< GraspPtr >& grasps = std::vector< GraspPtr >());

        /*!
        */
        virtual ~GraspSet();

        std::string getName();
        std::string getRobotType();
        std::string getEndEffector();


        void addGrasp(GraspPtr grasp);
        bool hasGrasp(GraspPtr grasp);
        bool hasGrasp(const std::string& name);
        bool removeGrasp(GraspPtr grasp);
        void removeAllGrasps();
        bool isCompatibleGrasp(GraspPtr grasp);
        void clear();

        /*!
            Return number of grasps stored in this set.
        */
        unsigned int getSize();

        /*!
            Return grasp number n. If n is out of bounds an empty GraspPtr is returned.
        */
        GraspPtr getGrasp(unsigned int n);
        GraspPtr getGrasp(const std::string& name);

        void print();

        std::string getXMLString(int tabs = 1);

        GraspSetPtr clone();

        std::vector< GraspPtr > getGrasps();

        //! Sets preshape string of all grasps
        void setPreshape(const std::string& preshape);

    protected:
        std::vector< GraspPtr > grasps;
        std::string name;
        std::string robotType;
        std::string eef;

    };

} // namespace

#endif // _VirtualRobot_GraspSet_h_
