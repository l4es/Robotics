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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010,2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_VisualizationNode_h_
#define _VirtualRobot_VisualizationNode_h_

#include "../VirtualRobotImportExport.h"
#include "VisualizationFactory.h"
#include <map>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace VirtualRobot
{

    class TriMeshModel;

    class VIRTUAL_ROBOT_IMPORT_EXPORT VisualizationNode
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        std::vector<Primitive::PrimitivePtr> primitives;

        /*!
        Constructor
        */
        VisualizationNode();

        /*!
        */
        virtual ~VisualizationNode();

        /*!
            Creates a triangulated model.
        */
        virtual TriMeshModelPtr getTriMeshModel();

        /*!
            Sets the position of the internal data structure.
        */
        virtual void setGlobalPose(const Eigen::Matrix4f& m)
        {
            globalPose = m;
        }
        inline const Eigen::Matrix4f& getGlobalPose() const
        {
            return globalPose;
        }

        /*!
            Clone this visualization.
            \param deepCopy When true, the underlying visualization is copied, otherwise a reference to the existing visualization is passed.
            \param scaling Scale Can be set to create a scaled version of this visual data.
            Since the underlying implementation may be able to re-use the visualization data, a deep copy may not be necessary in some cases.
        */
        virtual VisualizationNodePtr clone(bool deepCopy = true, float scaling = 1.0f);

        /*!
            Attach an optional visualization to this VisualizationNode. The attached visualizations will not show up in the TriMeshModel.
            If there is already a visualization attached with the given name, it is quietly replaced.
        */
        virtual void attachVisualization(const std::string& name, VisualizationNodePtr v);

        /*!
            Remove an attached visualization.
        */
        virtual void detachVisualization(const std::string& name);

        /*!
            Check for an attached visualization.
        */
        virtual bool hasAttachedVisualization(const std::string& name);

        /*!
            Get for an attached visualization.
        */
        virtual VisualizationNodePtr getAttachedVisualization(const std::string& name);

        /*!
            Setup the visualization of this object.
            \param showVisualization If false, the visualization is disabled.
            \param showAttachedVisualizations If false, the visualization of any attached optional visualizations is disabled.
        */
        virtual void setupVisualization(bool showVisualization, bool showAttachedVisualizations);

        /*!
            Enables/Disables the visualization updates.
            Usually if a SceneObject or a RobotNode changes its state, the visualization is automatically updated.
            This behavior can be changed here.
        */
        void setUpdateVisualization(bool enable);
        bool getUpdateVisualizationStatus();


        //! print information about this visualization object.
        virtual void print();

        //! get number of faces (i.e. triangles) of this object
        virtual int getNumFaces();

        //! optional filename tag
        std::string getFilename();
        bool usedBoundingBoxVisu();

        //! Just stores the filename, no loading is performed!
        void setFilename(const std::string& filename, bool boundingBox);

        virtual std::string getType()
        {
            return VisualizationFactory::getName();
        }

        std::string toXML(const std::string& basePath, int tabs);

        /*!
            Ctreate XML string and replace filename
        */
        std::string toXML(const std::string& basePath, const std::string& filename, int tabs);

        /*!
            Create a united visualization. Behavior depends on the derived implementation,
            but usually the visualizations are copied and united to one object.
        */
        static VisualizationNodePtr CreateUnitedVisualization(const std::vector<VisualizationNodePtr>& visualizations);

        /*!
            Returns (current) bounding box in global coordinate system.
        */
        BoundingBox getBoundingBox();

        /*!
            Saves model file to model path.
            \param modelPath The directory.
        */
        virtual bool saveModel(const std::string& modelPath, const std::string& filename);

        virtual void scale(Eigen::Vector3f& scaleFactor);

        //! update trimesh model
        virtual void createTriMeshModel();

    protected:
        bool boundingBox; //!< Indicates, if the bounding box model was used
        std::string filename; //!< if the visualization was build from a file, the filename is stored here

        Eigen::Matrix4f globalPose;
        bool updateVisualization;

        bool showVisualization;
        bool showAttachedVisualizations;

        std::map< std::string, VisualizationNodePtr > attachedVisualizations;   //< These optional visualizations will not show up in the TriMeshModel
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_VisualizationNode_h_
