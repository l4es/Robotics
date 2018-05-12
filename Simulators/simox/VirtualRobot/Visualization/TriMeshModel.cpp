/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/

#include "TriMeshModel.h"
#include "../VirtualRobot.h"

#include<Eigen/Geometry>

#include <algorithm>
#include <fstream>
#include <iomanip>
namespace VirtualRobot
{


    TriMeshModel::TriMeshModel()
    {
    }

    TriMeshModel::TriMeshModel(std::vector <triangle> &triangles)
    {
        for (size_t i = 0; i < triangles.size(); i++)
            addTriangleWithFace(triangles[i].vertex1, triangles[i].vertex2, triangles[i].vertex3);
    }

    /**
     * This method adds the vertices \p vertex1,
     * \p vertex2 and \p vertex3 to TriMeshModel::vertices and creates a new
     * TriangleFace instance which is added to TriMeshModel::faces.
     *
     * \param vertex1 first vertex to use in the calculation
     * \param vertex2 second vertex to use in the calculation
     * \param vertex3 third vertex to use in the calculation
     */
    void TriMeshModel::addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3)
    {
        Eigen::Vector3f normal = TriMeshModel::CreateNormal(vertex1, vertex2, vertex3);
        addTriangleWithFace(vertex1, vertex2, vertex3, normal);
    }

    void TriMeshModel::addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3, Eigen::Vector3f& normal, VisualizationFactory::Color color1, VisualizationFactory::Color color2, VisualizationFactory::Color color3)
    {
        this->addVertex(vertex1);
        this->addVertex(vertex2);
        this->addVertex(vertex3);

        this->addColor(color1);
        this->addColor(color2);
        this->addColor(color3);

        if (normal.norm() < 1e-10)
        {
            normal = TriMeshModel::CreateNormal(vertex1, vertex2, vertex3);
        }
        else
        {
            normal.normalize();
        }

        // create face
        MathTools::TriangleFace face;
        face.id1 = this->vertices.size() - 3;
        face.id2 = this->vertices.size() - 2;
        face.id3 = this->vertices.size() - 1;

        face.idColor1 = this->colors.size() - 3;
        face.idColor2 = this->colors.size() - 2;
        face.idColor3 = this->colors.size() - 1;

        face.normal = normal;

        this->addFace(face);
    }

    void TriMeshModel::addTriangleWithFace(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3, Eigen::Vector4f& vertexColor1, Eigen::Vector4f& vertexColor2, Eigen::Vector4f& vertexColor3)
    {
        Eigen::Vector3f normal = TriMeshModel::CreateNormal(vertex1, vertex2, vertex3);
        VisualizationFactory::Color color1(vertexColor1(0), vertexColor1(1), vertexColor1(2), vertexColor1(4));
        VisualizationFactory::Color color2(vertexColor2(0), vertexColor2(1), vertexColor2(2), vertexColor2(4));
        VisualizationFactory::Color color3(vertexColor3(0), vertexColor3(1), vertexColor3(2), vertexColor3(4));
        addTriangleWithFace(vertex1, vertex2, vertex3, normal, color1, color2, color3);
    }


    /**
     * This method creates the normal belonging to the vertices \p vertex1,
     * \p vertex2 and \p vertex3.
     *
     * \param vertex1 first vertex to use in the calculation
     * \param vertex2 second vertex to use in the calculation
     * \param vertex3 third vertex to use in the calculation
     *
     * \return normal vector
     */
    Eigen::Vector3f TriMeshModel::CreateNormal(Eigen::Vector3f& vertex1, Eigen::Vector3f& vertex2, Eigen::Vector3f& vertex3)
    {
        static bool warningPrinted = false;
        // calculate normal
        Eigen::Vector3f v1v2 = vertex2 - vertex1;
        Eigen::Vector3f v1v3 = vertex3 - vertex1;
        Eigen::Vector3f normal = v1v2.cross(v1v3);

        float l = normal.norm();

        if (l < 1e-10 && !warningPrinted)
        {
            VR_INFO << ": Warning: tiny normal found in TriMeshModel. This error is printed only once!\n";
            warningPrinted = true;
        }
        else
        {
            normal /= l;
        }

        return normal;
    }


    /**
     * This method adds a face to the internal data structure TriMeshModel::faces.
     */
    void TriMeshModel::addFace(const MathTools::TriangleFace& face)
    {
        faces.push_back(face);
    }


    /**
    * This method adds a vertex to the internal data structure TriMeshModel::vertices.
    */
    void TriMeshModel::addVertex(const Eigen::Vector3f& vertex)
    {
        vertices.push_back(vertex);
        boundingBox.addPoint(vertex);
    }

    /**
    * This method adds a normal to the internal data structure TriMeshModel::normals.
    */
    void TriMeshModel::addNormal(const Eigen::Vector3f& normal)
    {
        normals.push_back(normal);
    }

    /**
     * This method adds a color to the internal data structure TriMeshModel::colors
     */
    void TriMeshModel::addColor(const VisualizationFactory::Color& color)
    {
        colors.push_back(color);
    }

    /**
     * This method converts and adds a color to the internal data structure TriMeshModel::colors
     */
    void TriMeshModel::addColor(const Eigen::Vector4f& color)
    {
        addColor(VisualizationFactory::Color(color(0), color(1), color(2), color(3)));
    }

    /**
     * This method converts and adds a color to the internal data structure TriMeshModel::materials
     */
    void TriMeshModel::addMaterial(const VisualizationFactory::PhongMaterial& material)
    {
        materials.push_back(material);
    }


    /**
     * This method clears the internal data structures TriMeshModel::faces and
     * TriMeshModel::vertices.
     */
    void TriMeshModel::clear()
    {
        vertices.clear();
        colors.clear();
        faces.clear();
        materials.clear();
        boundingBox.clear();
    }


    /**
     * This method calls TriangleFace::flipOrientation() on each entry in
     * TriMeshModel::faces.
     */
    void TriMeshModel::flipVertexOrientations()
    {
        std::for_each(faces.begin(), faces.end(), std::mem_fun_ref(&MathTools::TriangleFace::flipOrientation));
    }


    /**
     * This method calculates the center of mass by accumulating all vertices and
     * dividing the sum by the number of vertices.
     */
    Eigen::Vector3f TriMeshModel::getCOM()
    {
        Eigen::Vector3f centerOfMass = Eigen::Vector3f::Zero();

        // accumulate all vertices
        std::vector<Eigen::Vector3f>::size_type i = 0;

        for (; i < vertices.size(); i++)
        {
            centerOfMass += vertices[i];
        }

        // divide by the number of vertices
        if (!vertices.empty())
        {
            centerOfMass /= (float)vertices.size();
        }

        return centerOfMass;
    }

    bool TriMeshModel::getSize(Eigen::Vector3f& storeMinSize, Eigen::Vector3f& storeMaxSize)
    {
        if (vertices.size() == 0)
        {
            return false;
        }

        storeMinSize = vertices[0];
        storeMaxSize = vertices[0];

        // go through all vertices
        std::vector<Eigen::Vector3f>::size_type i = 0;

        for (; i < vertices.size(); i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (vertices[i][j] < storeMinSize[j])
                {
                    storeMinSize[j] = vertices[i][j];
                }

                if (vertices[i][j] > storeMaxSize[j])
                {
                    storeMaxSize[j] = vertices[i][j];
                }
            }
        }

        return true;
    }

    /**
     * This method checks if the faces \p face1 and \p face2 share one common edge.
     *
     * \param face1 first TriangleFace to use in comparison
     * \param face2 second TriangleFace to use in comparison
     * \param commonVertexIds contains a list of
     *
     * \return true if both faces share the same edge and false if not
     */
    bool TriMeshModel::checkFacesHaveSameEdge(const MathTools::TriangleFace& face1, const MathTools::TriangleFace& face2, std::vector<std::pair<int, int> >& commonVertexIds) const
    {
        commonVertexIds.clear();
        Eigen::Vector2i vertexIds;
        const Eigen::Vector3f& face1Id1 = vertices[face1.id1];
        const Eigen::Vector3f& face1Id2 = vertices[face1.id2];
        const Eigen::Vector3f& face1Id3 = vertices[face1.id3];
        const Eigen::Vector3f& face2Id1 = vertices[face2.id1];
        const Eigen::Vector3f& face2Id2 = vertices[face2.id2];
        const Eigen::Vector3f& face2Id3 = vertices[face2.id3];

        // compare id1 of face1 with all Ids of face2
        // and add a pair of the indices to commonVertexIds if they match
        if (face1Id1 == face2Id1)
        {
            commonVertexIds.push_back(std::make_pair(1, 1));
        }

        if (face1Id1 == face2Id2)
        {
            commonVertexIds.push_back(std::make_pair(1, 2));
        }

        if (face1Id1 == face2Id3)
        {
            commonVertexIds.push_back(std::make_pair(1, 3));
        }

        // compare id2 of face1 with all Ids of face2
        // and add a pair of the indices to commonVertexIds if they match
        if (face1Id2 == face2Id1)
        {
            commonVertexIds.push_back(std::make_pair(2, 1));
        }

        if (face1Id2 == face2Id2)
        {
            commonVertexIds.push_back(std::make_pair(2, 2));
        }

        if (face1Id2 == face2Id3)
        {
            commonVertexIds.push_back(std::make_pair(2, 3));
        }

        // compare id3 of face1 with all Ids of face2
        // and add a pair of the indices to commonVertexIds if they match
        if (face1Id3 == face2Id1)
        {
            commonVertexIds.push_back(std::make_pair(3, 1));
        }

        if (face1Id3 == face2Id2)
        {
            commonVertexIds.push_back(std::make_pair(3, 2));
        }

        if (face1Id3 == face2Id3)
        {
            commonVertexIds.push_back(std::make_pair(3, 3));
        }

        // if both faces share 2 vertices they also share
        // one edge which goes from one vertex to the other
        return (2 == commonVertexIds.size());
    }


    /**
     * This method checks if all normals of the model point inwards or outwards and
     * flippes the faces which have a wrong orientation.
     * \param inverted inverts the check if set to true
     * \return the number of flipped faces
     */
    unsigned int TriMeshModel::checkAndCorrectNormals(bool inverted)
    {
        MathTools::TriangleFace* f1, *f2;
        int a1, a2, b1, b2;
        int flippedFacesCount = 0;

        // compare each vertex with every other vertex
        for (unsigned int i = 0; i < faces.size(); i++)
        {
            f1 = &(faces[i]);

            for (unsigned int j = 0; j < faces.size(); j++)
            {
                // don't compare each face with itself
                if (i == j)
                {
                    continue;
                }

                f2 = &(faces[j]);
                std::vector<std::pair<int, int> > commonVertexIds;

                if (checkFacesHaveSameEdge(*f1, *f2, commonVertexIds))
                {
                    a1 = commonVertexIds[0].first; // first common vertex id face1
                    a2 = commonVertexIds[1].first; // second common vertex id face1
                    b1 = commonVertexIds[0].second; // first common vertex id face
                    b2 = commonVertexIds[1].second; // second common vertex id face2
                    bool bAok = ((a1 == 1 && a2 == 2) || (a1 == 2 && a2 == 3) || (a1 == 3 && a2 == 1));
                    bool bBok = ((b1 == 1 && b2 == 3) || (b1 == 3 && b2 == 2) || (b1 == 2 && b2 == 1));

                    if (inverted)
                    {
                        bAok = !bAok;
                    }

                    // if both faces are not oriented the same flip f2
                    if (bAok && !bBok)
                    {
                        flippedFacesCount++;
                        f2->flipOrientation();
                    }
                    else if (!bAok &&  bBok)
                    {
                        flippedFacesCount++;
                        f2->flipOrientation();
                    }
                }
            }
        }

        return flippedFacesCount;
    }

    void TriMeshModel::print()
    {
        cout << "TriMeshModel\nNr of Faces:" << faces.size() << "\nNr of vertices:" << vertices.size() << endl;
        cout << "Normals:" << endl;


        std::streamsize pr = cout.precision(2);

        for (size_t i = 0; i < faces.size(); i++)
        {
            cout << "<" << faces[i].normal(0) << "," << faces[i].normal(1) << "," << faces[i].normal(2) << ">,";
        }

        cout << endl;
        boundingBox.print();
        cout.precision(pr);
    }

    void TriMeshModel::scale(Eigen::Vector3f& scaleFactor)
    {
        if (scaleFactor(0) == 1.0f && scaleFactor(1) == 1.0f && scaleFactor(2) == 1.0f)
        {
            return;
        }

        for (size_t i = 0; i < vertices.size(); i++)
        {
            for (int j = 0; j < 3; j++)
            {
                vertices[i][j] *= scaleFactor(j);
            }
        }

        boundingBox.scale(scaleFactor);
    }

    VirtualRobot::TriMeshModelPtr TriMeshModel::clone()
    {
        Eigen::Vector3f scaleFactor;
        scaleFactor << 1.0f, 1.0f, 1.0f;
        return clone(scaleFactor);
    }

    VirtualRobot::TriMeshModelPtr TriMeshModel::clone(Eigen::Vector3f& scaleFactor)
    {
        TriMeshModelPtr r(new TriMeshModel());
        r->vertices = vertices;
        r->faces = faces;
        r->boundingBox = boundingBox;
        r->scale(scaleFactor);
        return r;
    }


} // namespace VirtualRobot
