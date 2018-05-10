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
#ifndef _VirtualRobot_Scene_h_
#define _VirtualRobot_Scene_h_

#include "VirtualRobotImportExport.h"
#include "SceneObject.h"
#include "Robot.h"
#include "RobotConfig.h"
#include "Nodes/RobotNode.h"
#include "Obstacle.h"
#include "Trajectory.h"
#include "ManipulationObject.h"
#include "RobotConfig.h"
#include <string>
#include <vector>
#include <Eigen/Core>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT Scene
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
        */
        Scene(const std::string& name);

        /*!
        */
        virtual ~Scene();

        /*!
            Registers the robot to this scene. If a robot with the same name is already registered nothing happens.
        */
        void registerRobot(RobotPtr robot);

        /*!
            Removes the robot to from this scene. If the robot is not registered nothing happens.
        */
        void deRegisterRobot(RobotPtr robot);
        void deRegisterRobot(const std::string& name);

        bool hasRobot(RobotPtr robot) const;
        bool hasRobot(const std::string& name) const;

        RobotPtr getRobot(const std::string& name);

        std::vector< RobotPtr > getRobots();


        /*!
            Registers the RobotConfig to this scene. If a config  with the same name is already registered nothing happens.
        */
        void registerRobotConfig(RobotPtr robot, RobotConfigPtr config);
        void registerRobotConfig(RobotPtr robot, std::vector<RobotConfigPtr> configs);

        /*!
            Removes the RobotConfig to from this scene. If the RobotConfig is not registered nothing happens.
        */
        void deRegisterRobotConfig(RobotPtr robot, RobotConfigPtr config);
        void deRegisterRobotConfig(RobotPtr robot, const std::string& name);

        bool hasRobotConfig(RobotPtr robot, RobotConfigPtr config);
        bool hasRobotConfig(RobotPtr robot, const std::string& name);

        RobotConfigPtr getRobotConfig(const std::string& robotName, const std::string& name);
        RobotConfigPtr getRobotConfig(RobotPtr robot, const std::string& name);

        std::vector< RobotConfigPtr > getRobotConfigs(RobotPtr robot);

        /*!
            Registers the ManipulationObject to this scene. If an ManipulationObject with the same name is already registered nothing happens.
        */
        void registerManipulationObject(ManipulationObjectPtr obj);

        /*!
            Removes the ManipulationObject to from this scene. If the ManipulationObject is not registered nothing happens.
        */
        void deRegisterManipulationObject(ManipulationObjectPtr obj);
        void deRegisterManipulationObject(const std::string& name);

        bool hasManipulationObject(ManipulationObjectPtr obstacle) const;
        bool hasManipulationObject(const std::string& name) const;

        ManipulationObjectPtr getManipulationObject(const std::string& name);

        std::vector< ManipulationObjectPtr > getManipulationObjects();

        /*!
            Registers the obstacle to this scene. If an obstacle with the same name is already registered nothing happens.
        */
        void registerObstacle(ObstaclePtr obstacle);

        /*!
            Removes the obstacle to from this scene. If the obstacle is not registered nothing happens.
        */
        void deRegisterObstacle(ObstaclePtr obstacle);
        void deRegisterObstacle(const std::string& name);

        bool hasObstacle(ObstaclePtr obstacle) const;
        bool hasObstacle(const std::string& name) const;

        ObstaclePtr getObstacle(const std::string& name);

        std::vector< ObstaclePtr > getObstacles();


        /*!
            Registers the Trajectory to this scene. If an Trajectory with the same name is already registered nothing happens.
        */
        void registerTrajectory(TrajectoryPtr t);

        /*!
            Removes the Trajectory to from this scene. If the Trajectory is not registered nothing happens.
        */
        void deRegisterTrajectory(TrajectoryPtr t);
        void deRegisterTrajectory(const std::string& name);

        bool hasTrajectory(TrajectoryPtr t) const;
        bool hasTrajectory(const std::string& name) const;

        TrajectoryPtr getTrajectory(const std::string& name);

        std::vector< TrajectoryPtr > getTrajectories();
        std::vector< TrajectoryPtr > getTrajectories(const std::string& robotName);




        /*!
            Registers the set to this scene. If a set with the same name is already registered nothing happens.
        */
        void registerSceneObjectSet(SceneObjectSetPtr sos);

        /*!
            Removes the set to from this scene. If the set is not registered nothing happens.
        */
        void deRegisterSceneObjectSet(SceneObjectSetPtr sos);
        void deRegisterSceneObjectSet(const std::string& name);

        bool hasSceneObjectSet(SceneObjectSetPtr sos) const;
        bool hasSceneObjectSet(const std::string& name) const;

        SceneObjectSetPtr getSceneObjectSet(const std::string& name);

        std::vector< SceneObjectSetPtr > getSceneObjectSets();





        RobotNodeSetPtr getRobotNodeSet(const std::string& robot, const std::string rns);

        std::string getName() const;

        /*!
            Retrieve a visualization in the given format.
            Example usage:
             boost::shared_ptr<VirtualRobot::CoinVisualization> visualization = scene->getVisualization<CoinVisualization>();
             SoNode* visualisationNode = NULL;
             if (visualization)
                 visualisationNode = visualization->getCoinVisualization();
        */
        template <typename T> boost::shared_ptr<T> getVisualization(SceneObject::VisualizationType visuType = SceneObject::Full, bool addRobots = true, bool addObstacles = true, bool addManipulationObjects = true, bool addTrajectories = true, bool addSceneObjectSets = true);



        /*!
            Creates an XML string that describes this scene.
            \param basePath All paths to robots or objects are stored relative to this path.
            \return The xml string.
        */
        std::string getXMLString(const std::string& basePath);
    protected:

        std::string name;

        std::vector< RobotPtr > robots;
        std::map< RobotPtr, std::vector< RobotConfigPtr > > robotConfigs;
        std::vector< ObstaclePtr > obstacles;
        std::vector< ManipulationObjectPtr > manipulationObjects;
        std::vector< SceneObjectSetPtr > sceneObjectSets;
        std::vector< TrajectoryPtr > trajectories;

    };

    /**
     * This method collects all visualization nodes and creates a new Visualization
     * subclass which is given by the template parameter T.
     * T must be a subclass of VirtualRobot::Visualization.
     * A compile time error is thrown if a different class type is used as template argument.
     */
    template <typename T>
    boost::shared_ptr<T> Scene::getVisualization(SceneObject::VisualizationType visuType, bool addRobots, bool addObstacles, bool addManipulationObjects, bool addTrajectories, bool addSceneObjectSets)
    {
        const bool IS_SUBCLASS_OF_VISUALIZATION = ::boost::is_base_of<Visualization, T>::value;
        BOOST_MPL_ASSERT_MSG(IS_SUBCLASS_OF_VISUALIZATION, TEMPLATE_PARAMETER_FOR_VirtualRobot_getVisualization_MUST_BT_A_SUBCLASS_OF_VirtualRobot__Visualization, (T));
        std::vector<VisualizationNodePtr> collectedVisualizationNodes;

        if (addRobots)
        {
            std::vector<VirtualRobot::RobotPtr> collectedRobots = getRobots();
            // collect all robotnodes
            std::vector<VirtualRobot::RobotNodePtr> collectedRobotNodes;

            for (size_t i = 0; i < collectedRobots.size(); i++)
            {
                collectedRobots[i]->getRobotNodes(collectedRobotNodes, false);
            }

            for (size_t i = 0; i < collectedRobotNodes.size(); i++)
            {
                collectedVisualizationNodes.push_back(collectedRobotNodes[i]->getVisualization(visuType));
            }
        }

        if (addObstacles)
        {
            std::vector<VirtualRobot::ObstaclePtr> collectedObstacles = getObstacles();

            for (size_t i = 0; i < collectedObstacles.size(); i++)
            {
                collectedVisualizationNodes.push_back(collectedObstacles[i]->getVisualization(visuType));
            }
        }

        if (addManipulationObjects)
        {
            std::vector<VirtualRobot::ManipulationObjectPtr> collectedManipulationObjects = getManipulationObjects();

            for (size_t i = 0; i < collectedManipulationObjects.size(); i++)
            {
                collectedVisualizationNodes.push_back(collectedManipulationObjects[i]->getVisualization(visuType));
            }
        }

        if (addTrajectories)
        {
            std::vector<VirtualRobot::TrajectoryPtr> collectedTrajectories = getTrajectories();

            for (size_t i = 0; i < collectedTrajectories.size(); i++)
            {
                collectedVisualizationNodes.push_back(collectedTrajectories[i]->getVisualization(T::getFactoryName()));
            }
        }

        if (addSceneObjectSets)
        {
            std::vector<VirtualRobot::SceneObjectSetPtr> collectedSceneObjectSets = getSceneObjectSets();

            for (size_t i = 0; i < collectedSceneObjectSets.size(); i++)
            {
                std::vector< SceneObjectPtr > sos = collectedSceneObjectSets[i]->getSceneObjects();

                for (size_t j = 0; j < sos.size(); j++)
                {
                    collectedVisualizationNodes.push_back(sos[j]->getVisualization(visuType));
                }
            }
        }

        boost::shared_ptr<T> visualization(new T(collectedVisualizationNodes));
        return visualization;
    }

} // namespace

#endif // _VirtualRobot_Scene_h_
