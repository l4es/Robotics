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
* @author     Kai Welke, Nikolaus Vahrenkamp
* @copyright  2011 Kai Welke, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _VR_GRAPH_GENERATOR_H_
#define _VR_GRAPH_GENERATOR_H_

#include "VirtualRobotImportExport.h"
#include "MathTools.h"
#include <vector>
#include <map>
#include <float.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace VirtualRobot
{

    /*!
        This class can be used to generate an approximated sphere representation.
        The sphere is represented by a set of uniformly sized triangles.

        You can use the method generateGraph to fill the data structure SphereApproximation.

    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT SphereApproximator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SphereApproximator() {};
        ~SphereApproximator() {};

        enum EPolyhedronType
        {
            eTetrahedron,
            eOctahedron,
            eIcosahedron
        };

        struct FaceIndex
        {
            std::vector<int> faceIds;
        };

        //! A data structure that represents a sphere
        struct SphereApproximation
        {
            std::vector< Eigen::Vector3f >  vertices;
            std::vector< MathTools::TriangleFace >  faces;
            std::map<int, FaceIndex> mapVerticeIndxToFaceIndx;
        };

        /*!
            Generates a sphere representation and fills the data structure storeResult with the result.
        */
        void generateGraph(SphereApproximation& storeResult, EPolyhedronType baseType, int levels, float radius);

        TriMeshModelPtr generateTriMesh(const SphereApproximation& a);

        int findVertex(const Eigen::Vector3f& position, float epsilon, std::vector<Eigen::Vector3f>& vertices);
        float AngleVecVec(const Eigen::Vector3f& vector1, const Eigen::Vector3f& vector2);

        bool check_intersect_tri(const Eigen::Vector3f& pt1, const Eigen::Vector3f& pt2, const Eigen::Vector3f& pt3, const Eigen::Vector3f& linept, const Eigen::Vector3f& vect, Eigen::Vector3f& storeIntersection);

    private:

        struct GraphData
        {
            std::vector<Eigen::Vector3f> vertices;
            std::vector<MathTools::TriangleFace> faces;
        };
        bool check_same_clock_dir(const Eigen::Vector3f& pt1, const Eigen::Vector3f& pt2, const Eigen::Vector3f& pt3, const Eigen::Vector3f& norm);

        void buildVertexFaceMapping(SphereApproximation& storeResult);
        void initBasePolyhedron(EPolyhedronType baseType, GraphData& gd);
        void generateTetrahedron(GraphData& gd);
        void generateOctahedron(GraphData& gd);
        void generateIcosahedron(GraphData& gd);
        void subDivide(GraphData& gd);
        void projectToSphere(float radius, GraphData& gd);
        void setVec(Eigen::Vector3f& v, float x, float y, float z);
        void addVecVec(const Eigen::Vector3f& vector1, const Eigen::Vector3f& vector2, Eigen::Vector3f& result);
        void mulVecScalar(const Eigen::Vector3f& vec, float scalar, Eigen::Vector3f& result);

    };

}

#endif /* _VR_GRAPH_GENERATOR_H_ */
