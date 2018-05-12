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
#ifndef _VirtualRobot_Obstacle_h_
#define _VirtualRobot_Obstacle_h_

#include "VirtualRobotImportExport.h"

#include <string>
#include <vector>

#include "CollisionDetection/CollisionModel.h"
#include "SceneObject.h"

namespace VirtualRobot
{
    /*!
        An obstacle is an object that owns a visualization and a collision model.
        It can be moved around and used for collision detection.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT Obstacle : public SceneObject
    {
    public:

        /*!
        */
        Obstacle(const std::string& name, VisualizationNodePtr visualization = VisualizationNodePtr(), CollisionModelPtr collisionModel = CollisionModelPtr(), const SceneObject::Physics& p = SceneObject::Physics(), CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        /*!
        */
        virtual ~Obstacle();

        virtual void print(bool printDecoration = true);

        /*!
            Clones this object. If no col checker is given, the one of the original object is used.
        */
        ObstaclePtr clone(const std::string& name, CollisionCheckerPtr colChecker = CollisionCheckerPtr())  const
        {
            return ObstaclePtr(_clone(name, colChecker));
        }

        int getID();

        /*!
            Create a standard obstacle.
            \param width The width of the box.
            \param height The height of the box.
            \param depth The depth of the box.
            \param color Specify the color.
            \param visualizationType Here the type of visualization can be specified (e.g. "Inventor"). If empty, the first registered visualization type (which is usually the only one) is used.
            \param colChecker Only needed if you plan to use the collision checker in parallel. If not given, the object is registered with the global singleton collision checker.
        */
        static ObstaclePtr createBox(float width, float height, float depth, VisualizationFactory::Color color = VisualizationFactory::Color::Red(), std::string visualizationType = "", CollisionCheckerPtr colChecker = CollisionCheckerPtr());
        /*!
            Create a standard obstacle.
            \param radius The radius of the sphere.
            \param color Specify the color.
            \param visualizationType Here the type of visualization can be specified (e.g. "Inventor"). If empty, the first registered visualization type (which is usually the only one) is used.
            \param colChecker Only needed if you plan to use the collision checker in parallel. If not given, the object is registered with the global singleton collision checker.
        */
        static ObstaclePtr createSphere(float radius, VisualizationFactory::Color color = VisualizationFactory::Color::Red(), std::string visualizationType = "", CollisionCheckerPtr colChecker = CollisionCheckerPtr());
        /*!
            Create a standard obstacle.
            \param radius The radius of the cylinder.
            \param height The height of the cylinder.
            \param color Specify the color.
            \param visualizationType Here the type of visualization can be specified (e.g. "Inventor"). If empty, the first registered visualization type (which is usually the only one) is used.
            \param colChecker Only needed if you plan to use the collision checker in parallel. If not given, the object is registered with the global singleton collision checker.
        */
        static ObstaclePtr createCylinder(float radius, float height, VisualizationFactory::Color color = VisualizationFactory::Color::Red(), std::string visualizationType = "", CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        /*!
        Create a standard obstacle from a mesh.
        \param mesh The mesh.
        \param visualizationType Here the type of visualization can be specified (e.g. "Inventor"). If empty, the first registered visualization type (which is usually the only one) is used.
        \param colChecker Only needed if you plan to use the collision checker in parallel. If not given, the object is registered with the global singleton collision checker.
        */
        static ObstaclePtr createFromMesh(TriMeshModelPtr mesh, std::string visualizationType = "", CollisionCheckerPtr colChecker = CollisionCheckerPtr());

        virtual std::string toXML(const std::string& basePath, int tabs = 0);

        void setFilename(const std::string& filename);
        std::string getFilename();

    protected:

        virtual Obstacle* _clone(const std::string& name, CollisionCheckerPtr colChecker = CollisionCheckerPtr()) const;

        std::string filename;

        // a counter for internal ids
        static int idCounter;
        // my id
        int id;
    };

} // namespace

#endif // _VirtualRobot_Obstacle_h_
