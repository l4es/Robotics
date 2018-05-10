/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2014 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_VirtualRobotMeshImportTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Import/MeshImport/STLReader.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>
#include <VirtualRobot/RuntimeEnvironment.h>

#include <string>
#include <sstream>

using namespace VirtualRobot;
using namespace std;

BOOST_AUTO_TEST_SUITE(VirtualRobotMeshImport)

BOOST_AUTO_TEST_CASE(testParseSTL)
{
    const char* stlcontent =
        "solid block100\n"
        "facet normal -1.000000e+000 0.000000e+000 0.000000e+000\n"
        "outer loop\n"
        "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
        "vertex 0.000000e+000 1.000000e+002 0.000000e+000\n"
        "vertex 0.000000e+000 0.000000e+000 1.000000e+002\n"
        "endloop\n"
        "endfacet\n"
        "facet normal -1.000000e+000 0.000000e+000 0.000000e+000\n"
        "outer loop\n"
        "vertex 0.000000e+000 0.000000e+000 1.000000e+002\n"
        "vertex 0.000000e+000 1.000000e+002 0.000000e+000\n"
        "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 0.000000e+000 0.000000e+000 1.000000e+000\n"
        "outer loop\n"
        "vertex 1.000000e+002 1.000000e+002 1.000000e+002\n"
        "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
        "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 0.000000e+000 0.000000e+000 1.000000e+000\n"
        "outer loop\n"
        "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
        "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
        "vertex 0.000000e+000 0.000000e+000 1.000000e+002\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 1.000000e+000 0.000000e+000 0.000000e+000\n"
        "outer loop\n"
        "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
        "vertex 1.000000e+002 1.000000e+002 1.000000e+002\n"
        "vertex 1.000000e+002 0.000000e+000 0.000000e+000\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 1.000000e+000 0.000000e+000 0.000000e+000\n"
        "outer loop\n"
        "vertex 1.000000e+002 0.000000e+000 0.000000e+000\n"
        "vertex 1.000000e+002 1.000000e+002 1.000000e+002\n"
        "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 0.000000e+000 0.000000e+000 -1.000000e+000\n"
        "outer loop\n"
        "vertex 0.000000e+000 1.000000e+002 0.000000e+000\n"
        "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
        "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 0.000000e+000 0.000000e+000 -1.000000e+000\n"
        "outer loop\n"
        "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
        "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
        "vertex 1.000000e+002 0.000000e+000 0.000000e+000\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 0.000000e+000 1.000000e+000 0.000000e+000\n"
        "outer loop\n"
        "vertex 1.000000e+002 1.000000e+002 1.000000e+002\n"
        "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
        "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 0.000000e+000 1.000000e+000 0.000000e+000\n"
        "outer loop\n"
        "vertex 0.000000e+000 1.000000e+002 1.000000e+002\n"
        "vertex 1.000000e+002 1.000000e+002 0.000000e+000\n"
        "vertex 0.000000e+000 1.000000e+002 0.000000e+000\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 0.000000e+000 -1.000000e+000 0.000000e+000\n"
        "outer loop\n"
        "vertex 1.000000e+002 0.000000e+000 0.000000e+000\n"
        "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
        "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
        "endloop\n"
        "endfacet\n"
        "facet normal 0.000000e+000 -1.000000e+000 0.000000e+000\n"
        "outer loop\n"
        "vertex 0.000000e+000 0.000000e+000 0.000000e+000\n"
        "vertex 1.000000e+002 0.000000e+000 1.000000e+002\n"
        "vertex 0.000000e+000 0.000000e+000 1.000000e+002\n"
        "endloop\n"
        "endfacet\n"
        "endsolid\n";
    std::string s(stlcontent);
    istringstream ss(s);
    STLReaderPtr r(new STLReader());
    TriMeshModelPtr t(new TriMeshModel());
    bool readSTLok = r->read(ss, STLReader::STLA, t);
    BOOST_REQUIRE(readSTLok);
    BOOST_REQUIRE(t);
    BOOST_CHECK_EQUAL(t->vertices.size(), 8); // identical vertices are mapped to one vertex instance
    BOOST_CHECK_EQUAL(t->faces.size(), 12);
    BOOST_CHECK_EQUAL(t->vertices.at(0)[0], 0.0f);
    BOOST_CHECK_EQUAL(t->vertices.at(0)[1], 100.0f);
    BOOST_CHECK_EQUAL(t->vertices.at(0)[2], 100.0f);
}

BOOST_AUTO_TEST_CASE(testLoadSTL)
{

    std::string filename = "objects/stl/Piggy6.stl";
    bool fileOK = RuntimeEnvironment::getDataFileAbsolute(filename);
    BOOST_REQUIRE(fileOK);

    STLReaderPtr r(new STLReader());
    TriMeshModelPtr t(new TriMeshModel());
    bool readSTLok = r->read(filename, t);
    BOOST_REQUIRE(readSTLok);
    BOOST_REQUIRE(t);
    BOOST_CHECK_GT(int(t->vertices.size()), 20);
    BOOST_CHECK_GT(int(t->faces.size()), 20);
}


BOOST_AUTO_TEST_SUITE_END()
