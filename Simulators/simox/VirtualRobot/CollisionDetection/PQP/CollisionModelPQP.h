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
#ifndef _VirtualRobot_CollisionModelPQP_h_
#define _VirtualRobot_CollisionModelPQP_h_

#include "../../VirtualRobotImportExport.h"

#include "../CollisionModelImplementation.h"

#include <string>
#include <vector>
#include <map>
#include <set>

#include "PQP++/PQP_Compile.h"
#include "PQP++/PQP.h"

namespace VirtualRobot
{

    class CollisionChecker;
    class CollisionCheckerPQP;

    /*!
        A PQP related implementation of a collision model.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CollisionModelPQP : public CollisionModelImplementation
    {
    public:
        friend class CollisionModel;

        /*!Standard Constructor
        Ptr If collision checks should be done in parallel, different CollisionCheckers can be specified.
        */
        CollisionModelPQP(TriMeshModelPtr modelData, CollisionCheckerPtr colChecker, int id);

        /*!Standard Destructor
        */
        virtual ~CollisionModelPQP();

        boost::shared_ptr<PQP::PQP_Model> getPQPModel()
        {
            return pqpModel;
        }

        virtual void print();

    protected:

        //! delete all data
        virtual void destroyData();
        void createPQPModel();

        boost::shared_ptr<PQP::PQP_Model> pqpModel;

        boost::shared_ptr<CollisionCheckerPQP> colCheckerPQP;
    };

} // namespace

#endif // _VirtualRobot_CollisionModelPQP_h_
