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
* @date       2011-02-24
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*
*
*/

#ifndef _VirtualRobot_CDManager_h_
#define _VirtualRobot_CDManager_h_

#include "../VirtualRobotImportExport.h"
#include "CollisionModel.h"
#include "../SceneObjectSet.h"
#include "CollisionChecker.h"

#include <vector>
#include <set>
#include <string>


namespace VirtualRobot
{
    /*!
    *
    * A framework that can handle different sets of collision models.
    * With a collision detection manager (cdm) multiple sets collision models can be specified for convenient collision
    * detection or distance calculations.
    * Two methods can be used to add collisionModelSets:
    * addCollisionModel(): All added sets are checked against each other.
    * addCollisionModelPair(): Only the specific pairs are checked.
    *
    * The methods can be safely mixed.
    *
    * @see CollsionModelSet
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CDManager
    {
    public:
        //! if pColChecker is not set, the global collision checker is used
        CDManager(CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        virtual ~CDManager();

        /*!
            Sets of SceneObjects can be added.
            All added SceneObjectSets sets are checked against each other.
            Internally for all SceneObjectSets that have been added earlier, the method addCollisionModelPair() is called.
        */
        void addCollisionModel(SceneObjectSetPtr m);

        /*!
            Here, a specific pair of SceneObjectSets can be added.
            m1 and m2 will only be checked against each other
            and will not be checked against the other SceneObjectSets, that may be part of this CDManager.
        */
        void addCollisionModelPair(SceneObjectSetPtr m1, SceneObjectSetPtr m2);
        void addCollisionModelPair(SceneObjectPtr m1, SceneObjectSetPtr m2);
        void addCollisionModelPair(SceneObjectPtr m1, SceneObjectPtr m2);

        /*!
            Here single collision models can be added. Internally they are wrapped by a SceneObjectSet.
        */
        void addCollisionModel(SceneObjectPtr m);


        bool hasSceneObjectSet(SceneObjectSetPtr m);
        bool hasSceneObject(SceneObjectPtr m);

        //! Returns true if one of the specified collision checks report a collision.
        bool isInCollision();

        /*!
            Checks if the model m collides with one of the added colModels. (all SceneObjectSets that have been added are considered)
            It is allowed to use an already added SceneObjectSets.
            Returns true if there is a collision.
        */
        bool isInCollision(SceneObjectSetPtr m);

        //! Returns minimum distance of all sets that have been added for consideration.
        float getDistance();

        /*!
            Calculates the shortest distance of m to all added SceneObjectSets.
            m may be an already added SceneObjectSets.
        */
        float getDistance(SceneObjectSetPtr m);

        //! Stores min dist position and collision IDs
        float getDistance(Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2);

        /*!
            Calculates the shortest distance of SceneObjectSet m to all added colModels.
            Stores nearest positions and corresponding IDs, where P1 and trID1 is used to store the data of m and
            P2 and trID2 is used to store the data of this CDManager.
        */
        float getDistance(SceneObjectSetPtr m, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2);


        //! All SceneObjectSets that have been added.
        std::vector<SceneObjectSetPtr> getSceneObjectSets();

        CollisionCheckerPtr getCollisionChecker();

    protected:
        /*!
            Performs also a check for sets with only one object added in order to cover potentionally added single SceneObjects.
        */
        bool _hasSceneObjectSet(SceneObjectSetPtr m);

        bool isInCollision(SceneObjectSetPtr m, std::vector<SceneObjectSetPtr>& sets);
        float getDistance(SceneObjectSetPtr m, std::vector<SceneObjectSetPtr>& sets, Eigen::Vector3f& P1, Eigen::Vector3f& P2, int& trID1, int& trID2);
        float getDistance(SceneObjectSetPtr m, std::vector<SceneObjectSetPtr>& sets);
        std::vector< SceneObjectSetPtr > colModels;
        CollisionCheckerPtr colChecker;

        std::map<SceneObjectSetPtr, std::vector<SceneObjectSetPtr> > colModelPairs;

    };

}

#endif // _VirtualRobot_CDManager_h_

