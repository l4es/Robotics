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
#ifndef _VirtualRobot_SceneObjectSet_h_
#define _VirtualRobot_SceneObjectSet_h_

#include "VirtualRobotImportExport.h"
#include <string>
#include <vector>
#include <map>
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace VirtualRobot
{

    class CollisionChecker;
    class CollisionModel;

    class VIRTUAL_ROBOT_IMPORT_EXPORT SceneObjectSet
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Standard Constructor.
            \param name A name string.
            \param colChecker Can be used to link this object to different instances of the collision checker.
            If NULL, the global collision checker singleton is used.
        */
        SceneObjectSet(const std::string& name = "", CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        /*!Standard Destructor.
        */
        virtual ~SceneObjectSet();

        /*! Returns name of this model
        */
        std::string getName() const;

        //! store axis aligned bounding box covering all CollisionModels to store_aabb
        //virtual void GetAABB(SbBox3f& store_aabb);

        //! store axis aligned bounding boxes to store_aabbs
        //virtual void GetAABBs(std::vector<SbBox3f>& store_aabbs);

        //! store object oriented bounding boxes of contained models to store_oobbs along with iv model
        //virtual void GetOOBBs(std::vector<SbXfBox3f>& store_oobbs);


        //! append a single object
        virtual bool addSceneObject(SceneObjectPtr sceneObject);

        //! append a set of collision models
        virtual bool addSceneObjects(SceneObjectSetPtr sceneObjectSet);

        /*!
            Append all associated object of the given robotNodeSet.
        */
        virtual bool addSceneObjects(RobotNodeSetPtr robotNodeSet);
        virtual bool addSceneObjects(std::vector<RobotNodePtr> robotNodes);

        //! remove a single col model from this Set
        virtual bool removeSceneObject(SceneObjectPtr sceneObject);

        CollisionCheckerPtr getCollisionChecker()
        {
            return colChecker;
        }

        /*!
            Returns all covered collision models.
        */
        std::vector< CollisionModelPtr > getCollisionModels();
        std::vector< SceneObjectPtr > getSceneObjects();

        virtual unsigned int getSize() const;
        virtual SceneObjectPtr getSceneObject(unsigned int nr);

        //! Returns true, if sceneObject is part of this set
        virtual bool hasSceneObject(SceneObjectPtr sceneObject);

        //! fills the current globalPose of all associated sceneobjects to map.
        virtual bool getCurrentSceneObjectConfig(std::map< SceneObjectPtr, Eigen::Matrix4f >& storeConfig);

        virtual std::string toXML(int tabs);

        /*!
            Create a (shallow) copy of this set. The containing sceneobjects are not cloned, but referenced by the newly generated set.
            \param newName The name of the newly created set.
         */
        SceneObjectSetPtr clone(const std::string& newName = "");

        /*!
            Create a deep copy of this set, which means that all sceneobjects are cloned.
            \param newName The name of the newly created set.
            \param newColChecker A new collision checker instance.
         */
        SceneObjectSetPtr clone(const std::string& newName, CollisionCheckerPtr newColChecker);

        /*!
            This method creates a new obstacle from all added SceneObjects.
            Note, that the resulting object is a rigid body that can be placed in the scene but the internal structure is fixed.
            This object can be useful when the SceneObjectSet consists of a large number of objects that do not change their relation to each other.
            When building a static obstacle collision detection can be performed much more efficient.
        */
        ObstaclePtr createStaticObstacle(const std::string& name);

    protected:

        //! delete all data
        void destroyData();

        std::string name;
        std::vector< SceneObjectPtr > sceneObjects;
        CollisionCheckerPtr colChecker;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_SceneObjectSet_h_
