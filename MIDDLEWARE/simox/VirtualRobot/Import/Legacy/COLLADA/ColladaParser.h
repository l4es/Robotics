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
 * @author     Stefan Ulbrich
 * @copyright  Stefan Ulbrich
 *             GNU Lesser General Public License
 *
 */
#include <iostream>
#include <dae.h>
#include <1.5/dom/domCOLLADA.h>
#include <1.5/dom/domInstance_kinematics_scene.h>
#include <1.5/dom/domTechnique.h>
#include <boost/foreach.hpp>
#include "ColladaIterators.h"
#include <boost/range/adaptors.hpp>
#include <boost/range/algorithm.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/shared_ptr.hpp>
#include <ostream>
#include "../VirtualRobotImportExport.h"

namespace VirtualRobot
{

    /** @brief A class that opens and parses a COLLADA File.
     *
     * Only a subset of the functionality of COLLADA is supported -- mainly the files created by
     * the OpenGRASP robot editor. Once the file is read, the kinematic and visual structure is
     * stored in a tree shaped data structure for further processing.
     */
    class VIRTUAL_ROBOT_IMPORT_EXPORT ColladaParser
    {
    public:

        /** @brief Class that represents the visual appearance of a robot node
         */
        struct SceneGraph
        {
            SceneGraph(unsigned int _level = 0) : level(_level) {};
            /// Representation of a mesh
            struct GeometryType
            {
                /// Material
                struct PhongMaterial
                {
                    std::string id;
                    std::vector<float> emission;
                    std::vector<float> ambient;
                    std::vector<float> diffuse;
                    std::vector<float> specular;
                    float shininess;
                    std::vector<float> reflective;
                    float reflectivity;
                    std::vector<float> transparent;
                    float transparency;
                    float refractionIndex;
                };

                std::vector<std::vector<float> > vertices;
                std::vector<std::vector<float> > normals;
                std::vector<std::vector<float> > colors;
                std::vector<PhongMaterial> materials;

                //these save the indices to vertices/normals/colors
                //each point of a triangle has its own normal/color
                std::vector<std::vector<unsigned int> > triangles;
                std::vector<std::vector<unsigned int> > triangleNormals;
                std::vector<std::vector<unsigned int> > triangleColors;

                //materials are saved on a per-polyList basis, but we throw them all together, hence the need for triangleMaterial below
                //material index of each triangle
                std::vector<unsigned int> triangleMaterial;
            };
            struct RigidBodyType
            {
                std::vector<std::vector<float> > mass_frame;
                float mass;
                std::vector<float> inertia;
            };

            /// Transformations before the mesh (4 floats represent rotations, 3 floats transformations).
            std::vector<std::vector<float> > transformations;
            std::vector<GeometryType> geometries;
            std::vector<RigidBodyType> rigidBodies;
            /// The visual appearance may be shaped as tree.
            std::vector<boost::shared_ptr<SceneGraph> > children;
            unsigned int level;
        };

        enum JointType
        {
            eNone,
            ePrismatic,
            eRevolute
        };

        /** @brief Graph structure that describes the kinematics of a robot
         *
         * An NodeData object later represents a robot node.
         */
        struct NodeData
        {
            NodeData(unsigned int _level = 0) : transformation_visual_scene(NULL), speed(0), acceleration(0), deceleration(0), jerk(0), active(false), locked(true), min(0), max(0), jointType(eNone), level(_level) {};
            unsigned int level;
            float speed, jerk, acceleration, deceleration;
            bool locked, active;
            float min, max, value;
            daeElement* transformation_visual_scene;
            std::string name;
            friend std::ostream& operator<<(std::ostream& os, const NodeData& dt);
            /// Transformations before the mesh (4 floats represent rotations, 3 floats transformations).
            std::vector<std::vector<float> > transformations;
            std::vector<float> axis;
            JointType jointType;
            std::vector<boost::shared_ptr<NodeData> > children;
            std::vector<boost::shared_ptr<NodeData> > sensors;
            boost::shared_ptr<SceneGraph> sceneGraph;

        };
        typedef std::map<std::string, boost::shared_ptr<NodeData> > ModelType;

        /// Returns a std::map that relates kinematic chain names to the extracted model structure.
        ModelType& getModel()
        {
            return this->model;
        };
        ColladaParser(std::string filename);
        void parse();
        ModelType model;
    private:
        typedef std::map<ColladaDOM150::domJoint*, boost::shared_ptr<NodeData> > NodeDataMap;
        ColladaDOM150::domLinkRef lastParent;
        DAE dae;
        void extractSensors(ColladaDOM150::domJoint* actualJoint, boost::shared_ptr<NodeData> jointNode, ColladaDOM150::domArticulated_system* motion_articulated_system);
        bool parseLink(ColladaDOM150::domLinkRef, boost::shared_ptr<NodeData>, ColladaDOM150::domKinematics_model*, NodeDataMap&);
        boost::shared_ptr<SceneGraph> parseVisualScene(ColladaDOM150::domNode* node, std::vector<daeElement*> delimiters, daeElement* start = NULL);
        std::map<ColladaDOM150::domNode*, ColladaParser::SceneGraph::RigidBodyType> rigid_body_map;
    };

    template<class T>
    std::ostream& operator<<(std::ostream& os, const daeTArray<T>& array);

    std::ostream& operator<<(std::ostream& os, const ColladaParser::NodeData& dt);

    std::ostream& operator<<(std::ostream& os, const ColladaParser::SceneGraph& dt);

    std::ostream& operator<<(std::ostream& os, const ColladaParser::SceneGraph::GeometryType& gt);

    template<class T>
    std::ostream& operator<<(std::ostream& os, const std::vector<std::vector<T> >& vec);

    template<class T>
    std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec);

}
