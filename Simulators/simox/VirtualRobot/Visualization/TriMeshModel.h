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
#ifndef _VirtualRobot_TriMeshModel_h_
#define _VirtualRobot_TriMeshModel_h_

#include "../VirtualRobotImportExport.h"
#include "../Visualization/VisualizationFactory.h"
#include "../MathTools.h"
#include "../BoundingBox.h"
#include <Eigen/Core>
#include <vector>
#include <utility>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT TriMeshModel
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        TriMeshModel();

        struct triangle
        {   
            Eigen::Vector3f vertex1;
            Eigen::Vector3f vertex2;
            Eigen::Vector3f vertex3;
        };
        TriMeshModel(std::vector <triangle> &triangles);

        void addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3);
        void addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3, Eigen::Vector3f& normal,
                                 VisualizationFactory::Color color1 = VisualizationFactory::Color::Gray(),
                                 VisualizationFactory::Color color2 = VisualizationFactory::Color::Gray(),
                                 VisualizationFactory::Color color3 = VisualizationFactory::Color::Gray());
        void addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3, Eigen::Vector4f& vertexColor1, Eigen::Vector4f& vertexColor2, Eigen::Vector4f& vertexColor3);
        static Eigen::Vector3f CreateNormal(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3);
        void addFace(const MathTools::TriangleFace& face);
        void addVertex(const Eigen::Vector3f& vertex);
        void addNormal(const Eigen::Vector3f& normal);
        void addColor(const VisualizationFactory::Color& color);
        void addColor(const Eigen::Vector4f& color);
        void addMaterial(const VisualizationFactory::PhongMaterial& material);
        void clear();
        void flipVertexOrientations();

        void print();
        Eigen::Vector3f getCOM();
        bool getSize(Eigen::Vector3f& storeMinSize, Eigen::Vector3f& storeMaxSize);
        bool checkFacesHaveSameEdge(const MathTools::TriangleFace& face1, const MathTools::TriangleFace& face2, std::vector<std::pair<int, int> >& commonVertexIds) const;
        unsigned int checkAndCorrectNormals(bool inverted);

        virtual void scale(Eigen::Vector3f& scaleFactor);
        TriMeshModelPtr clone();
        TriMeshModelPtr clone(Eigen::Vector3f& scaleFactor);

        std::vector<Eigen::Vector3f> normals;
        std::vector<Eigen::Vector3f> vertices;
        std::vector<VisualizationFactory::Color> colors;
        std::vector<MathTools::TriangleFace> faces;
        std::vector<VisualizationFactory::PhongMaterial> materials;
        BoundingBox boundingBox;
    };
} // namespace VirtualRobot

#endif /* _VirtualRobot_TriMeshModel_h_ */
