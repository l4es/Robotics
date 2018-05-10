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
#ifndef _VirtualRobot_VisualizationFactory_h_
#define _VirtualRobot_VisualizationFactory_h_

#include "../VirtualRobotImportExport.h"
#include "../AbstractFactoryMethod.h"
#include "../MathTools.h"
#include "../BoundingBox.h"
#include "../Primitive.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

namespace VirtualRobot
{
    class VisualizationNode;

    class VIRTUAL_ROBOT_IMPORT_EXPORT VisualizationFactory  : public AbstractFactoryMethod<VisualizationFactory, void*>
    {
    public:

        struct Color
        {
            Color()
            {
                transparency = 0.0f;
                r = g = b = 0.5f;
            }
            Color(float r, float g, float b, float transparency = 0.0f): r(r), g(g), b(b), transparency(transparency) {}
            float r, g, b;
            float transparency;
            bool isNone() const
            {
                return transparency >= 1.0f;
            }
            static Color Blue(float transparency = 0.0f)
            {
                return Color(0.2f, 0.2f, 1.0f, transparency);
            }
            static Color Red(float transparency = 0.0f)
            {
                return Color(1.0f, 0.2f, 0.2f, transparency);
            }
            static Color Green(float transparency = 0.0f)
            {
                return Color(0.2f, 1.0f, 0.2f, transparency);
            }
            static Color Black(float transparency = 0.0f)
            {
                return Color(0, 0, 0, transparency);
            }
            static Color Gray()
            {
                return Color(0.5f, 0.5f, 0.5f, 0);
            }
            static Color None()
            {
                return Color(0.0f, 0.0f, 0.0f, 1.0f);
            }
        };

        struct PhongMaterial
        {
            PhongMaterial() {}
            Color emission;
            Color ambient;
            Color diffuse;
            Color specular;
            float shininess;
            Color reflective;
            float reflectivity;
            Color transparent;
            float transparency;
            float refractionIndex;
        };

        VisualizationFactory()
        {
            ;
        }
        virtual ~VisualizationFactory()
        {
            ;
        }

        virtual VisualizationNodePtr getVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& primitives, bool boundingBox = false, Color color = Color::Gray())
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr getVisualizationFromFile(const std::string& filename, bool boundingBox = false)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr getVisualizationFromFile(const std::ifstream& ifs, bool boundingBox = false)
        {
            return VisualizationNodePtr();
        }
        /*!
            A box, dimensions are given in mm.
        */
        virtual VisualizationNodePtr createBox(float width, float height, float depth, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createLine(const Eigen::Vector3f& from, const Eigen::Vector3f& to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width = 1.0f, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createSphere(float radius, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createCylinder(float radius, float height, float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createCoordSystem(float scaling = 1.0f, std::string* text = NULL, float axisLength = 100.0f, float axisSize = 3.0f, int nrOfBlocks = 10)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createBoundingBox(const BoundingBox& bbox, bool wireFrame = false)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createVertexVisualization(const Eigen::Vector3f& position, float radius, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createTriMeshModelVisualization(TriMeshModelPtr model, Eigen::Matrix4f& pose)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createTriMeshModelVisualization(TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f& pose, bool showLines = true)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createPlane(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, float extend, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createPlane(const MathTools::Plane& plane, float extend, float transparency,  float colorR = 0.5f, float colorG = 0.5f, float colorB = 0.5f)
        {
            return createPlane(plane.p, plane.n, extend, transparency, colorR, colorG, colorB);
        }
        virtual VisualizationNodePtr createArrow(const Eigen::Vector3f& n, float length = 50.0f, float width = 2.0f, const Color& color = Color::Gray())
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createTrajectory(TrajectoryPtr t, Color colorNode = Color::Blue(), Color colorLine = Color::Gray(), float nodeSize = 15.0f, float lineSize = 4.0f)
        {
            return VisualizationNodePtr();
        }
        virtual VisualizationNodePtr createText(const std::string& text, bool billboard = false, float scaling = 1.0f, Color c = Color::Black(), float offsetX = 20.0f, float offsetY = 20.0f, float offsetZ = 0.0f)
        {
            return VisualizationNodePtr();
        }
        /*!
            Creates an coordinate axis aligned ellipse
            \param x The extend in x direction must be >= 1e-6
            \param y The extend in y direction must be >= 1e-6
            \param z The extend in z direction must be >= 1e-6
            \param showAxes If true, the axes are visualized
            \param axesHeight The height of the axes (measured from the body surface)
            \param axesWidth The width of the axes.
            \return A VisualizationNode containing the visualization.
        */
        virtual VisualizationNodePtr createEllipse(float x, float y, float z, bool showAxes = true, float axesHeight = 4.0f, float axesWidth = 8.0f)
        {
            return VisualizationNodePtr();
        }
        /*!
            Move local visualization by homogeneous matrix m.
        */
        virtual void applyDisplacement(VisualizationNodePtr o, Eigen::Matrix4f& m) {}

        /*!
            Create an empty VisualizationNode.
        */
        virtual VisualizationNodePtr createVisualization()
        {
            return VisualizationNodePtr();
        }

        /*!
            Create a united visualization.
        */
        virtual VisualizationNodePtr createUnitedVisualization(const std::vector<VisualizationNodePtr>& visualizations) const
        {
            return VisualizationNodePtr();
        }

        /*!
            Here, a manual cleanup can be called, no Coin3D access possible after this.
            Usually no need to call cleanup explicitly, since cleanup is performed automatically at application exit.
        */
        virtual void cleanup()
        {
            ;
        }

    };
    typedef boost::shared_ptr<VisualizationFactory::Color> ColorPtr;

} // namespace VirtualRobot

#endif // _VirtualRobot_VisualizationFactory_h_
