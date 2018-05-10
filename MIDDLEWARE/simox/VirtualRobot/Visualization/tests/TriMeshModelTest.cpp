/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/

#define BOOST_TEST_MODULE VirtualRobot_TriMeshModelTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <Eigen/Core>

BOOST_AUTO_TEST_SUITE(Visualization)

BOOST_AUTO_TEST_CASE(testTriangleFaceFlipOrientation)
{
    VirtualRobot::MathTools::TriangleFace face;
    face.id1 = 1;
    face.id2 = 2;
    face.id3 = 3;
    face.normal = Eigen::Vector3f::Ones();
    BOOST_CHECK_EQUAL(face.id1, 1);
    BOOST_CHECK_EQUAL(face.id2, 2);
    BOOST_CHECK_EQUAL(face.id3, 3);
    face.flipOrientation();
    BOOST_CHECK_EQUAL(face.id1, 3);
    BOOST_CHECK_EQUAL(face.id2, 2);
    BOOST_CHECK_EQUAL(face.id3, 1);
    BOOST_CHECK_EQUAL(face.normal.x(), -1);
    BOOST_CHECK_EQUAL(face.normal.y(), -1);
    BOOST_CHECK_EQUAL(face.normal.z(), -1);
}

BOOST_AUTO_TEST_CASE(testTriMeshModelCreation)
{
    VirtualRobot::TriMeshModel model;
    BOOST_CHECK_EQUAL(model.vertices.size(), 0);
    BOOST_CHECK_EQUAL(model.faces.size(), 0);
}


BOOST_AUTO_TEST_CASE(testAddTriangleWithFace)
{
    Eigen::Vector3f vertex1, vertex2, vertex3;
    vertex1 << 1, 0, 0;
    vertex2 << 0, 1, 0;
    vertex3 << 0, 0, 1;

    VirtualRobot::TriMeshModel model;
    model.addTriangleWithFace(vertex1, vertex2, vertex3);
    BOOST_CHECK_EQUAL(model.vertices.size(), 3);
    BOOST_CHECK_EQUAL(model.faces.size(), 1);
}


BOOST_AUTO_TEST_CASE(testAddVertex)
{
    Eigen::Vector3f vertex;
    vertex << 1, 0, 0;
    VirtualRobot::TriMeshModel model;
    const unsigned int numberOfFaces = model.faces.size();
    const unsigned int numberOfVertices = model.vertices.size();
    model.addVertex(vertex);
    BOOST_CHECK_EQUAL(model.vertices.size(), numberOfVertices + 1);
    BOOST_CHECK_EQUAL(model.faces.size(), numberOfFaces);
}


BOOST_AUTO_TEST_CASE(testAddFace)
{
    VirtualRobot::MathTools::TriangleFace face;
    VirtualRobot::TriMeshModel model;
    const unsigned int numberOfFaces = model.faces.size();
    const unsigned int numberOfVertices = model.vertices.size();
    model.addFace(face);
    BOOST_CHECK_EQUAL(model.vertices.size(), numberOfVertices);
    BOOST_CHECK_EQUAL(model.faces.size(), numberOfFaces  + 1);
}


BOOST_AUTO_TEST_CASE(testClear)
{
    Eigen::Vector3f vertex;
    vertex << 1, 0, 0;
    VirtualRobot::TriMeshModel model;
    model.addTriangleWithFace(vertex, vertex, vertex);
    BOOST_CHECK_GT(model.vertices.size(), (size_t)0);
    BOOST_CHECK_GT(model.faces.size(), (size_t)0);

    model.clear();
    BOOST_CHECK_EQUAL(model.vertices.size(), 0);
    BOOST_CHECK_EQUAL(model.faces.size(), 0);
}


BOOST_AUTO_TEST_CASE(testCreateNormal)
{
    Eigen::Vector3f vertex1, vertex2, vertex3;
    vertex1 << 1, 0, 0;
    vertex2 << 0, 1, 0;
    vertex3 << 0, 0, 1;

    Eigen::Vector3f normal = VirtualRobot::TriMeshModel::CreateNormal(vertex1, vertex2, vertex3);

    BOOST_CHECK_CLOSE(normal.x(), 0.5774, 0.01);
    BOOST_CHECK_CLOSE(normal.y(), 0.5774, 0.01);
    BOOST_CHECK_CLOSE(normal.z(), 0.5774, 0.01);
}


BOOST_AUTO_TEST_CASE(testFlipVertexOrientation)
{
    VirtualRobot::MathTools::TriangleFace face;
    face.id1 = 1;
    face.id2 = 2;
    face.id3 = 3;
    face.normal = Eigen::Vector3f::Ones();

    VirtualRobot::TriMeshModel model;
    model.addFace(face);
    model.addFace(face);

    model.flipVertexOrientations();

    std::vector<VirtualRobot::MathTools::TriangleFace>::iterator iter = model.faces.begin();

    while (iter != model.faces.end())
    {
        BOOST_CHECK_EQUAL(iter->id1, 3);
        BOOST_CHECK_EQUAL(iter->id2, 2);
        BOOST_CHECK_EQUAL(iter->id3, 1);
        BOOST_CHECK_EQUAL(iter->normal.x(), -1);
        BOOST_CHECK_EQUAL(iter->normal.y(), -1);
        BOOST_CHECK_EQUAL(iter->normal.z(), -1);
        ++iter;
    }
}

BOOST_AUTO_TEST_CASE(testGetCOM)
{
    Eigen::Vector3f v1, v2, v3, v4;
    v1 << 0, 0, 0;
    v2 << 1, 0, 0;
    v3 << 0, 1, 0;
    v4 << 0, 0, 1;
    VirtualRobot::TriMeshModel model;
    model.addVertex(v1);
    model.addVertex(v2);
    model.addVertex(v3);
    model.addVertex(v4);

    Eigen::Vector3f com = model.getCOM();
    BOOST_CHECK_CLOSE(com.x(), 0.25, 0.01);
    BOOST_CHECK_CLOSE(com.y(), 0.25, 0.01);
    BOOST_CHECK_CLOSE(com.z(), 0.25, 0.01);
}

BOOST_AUTO_TEST_SUITE_END()
