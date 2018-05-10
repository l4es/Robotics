/**
* @package    VirtualRobot
* @author    Nikolaus Vahrenkamp
* @copyright  2012 nv
*/

#define BOOST_TEST_MODULE VirtualRobot_VoxelTreeTest

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Workspace/VoxelTree6D.hpp>
#include <VirtualRobot/Workspace/VoxelTreeND.hpp>
#include <VirtualRobot/Workspace/VoxelTreeNDElement.hpp>

BOOST_AUTO_TEST_SUITE(VoxelTree)

BOOST_AUTO_TEST_CASE(VoxelTreeConstructionTest)
{
    float minB[6];
    float maxB[6];

    for (int i = 0; i < 3; i++)
    {
        minB[i] = -100.0f;
        maxB[i] = 100.0f;
        minB[3 + i] = (float) - M_PI;
        maxB[3 + i] = (float)M_PI;
    }

    BOOST_REQUIRE_NO_THROW(VirtualRobot::VoxelTree6D<unsigned char> v(minB, maxB, 20.0f, 1.0f));
}


BOOST_AUTO_TEST_CASE(VoxelTreeNDConstructionTest)
{
    float minB[5];
    float maxB[5];
    float discr[5];

    for (int i = 0; i < 4; i++)
    {
        minB[i] = -100.0f;
        maxB[i] = 100.0f;
        discr[i] = 20.0f;
    }

    minB[4] = 0.0f;
    maxB[4] = 1200.0f;
    discr[4] = 1.0f;
    VirtualRobot::VoxelTreeND<unsigned char, 5> v(minB, maxB, discr);
}


BOOST_AUTO_TEST_CASE(VoxelTreeEntriesTest)
{
    float minB[6];
    float maxB[6];

    for (int i = 0; i < 3; i++)
    {
        minB[i] = -100.0f;
        maxB[i] = 100.0f;
        minB[3 + i] = (float) - M_PI;
        maxB[3 + i] = (float)M_PI;
    }

    VirtualRobot::VoxelTree6D<unsigned char> v(minB, maxB, 20.0f, 1.0f);
    float pos[6];

    for (int i = 0; i < 6; i++)
    {
        pos[i] = 0;
    }

    unsigned char* c = v.getEntry(pos);
    bool isNull = (c == NULL);
    BOOST_REQUIRE(isNull);
    v.setEntry(pos, 10);
    c = v.getEntry(pos);
    isNull = (c == NULL);
    BOOST_REQUIRE(!isNull);

    BOOST_REQUIRE_EQUAL(*c, 10);

}

BOOST_AUTO_TEST_CASE(VoxelTreeNDEntriesTest)
{
    const unsigned int N = 6;
    float minB[N];
    float maxB[N];
    float discr[N];
    float extend = 2.0f;

    for (int i = 0; i < N; i++)
    {
        minB[i] = -100.0f;
        maxB[i] = 100.0f;
        discr[i] = extend * (i + 1); // different min extends
    }

    VirtualRobot::VoxelTreeND<unsigned char, N> v(minB, maxB, discr, true);
    float pos[N];

    for (int i = 0; i < N; i++)
    {
        pos[i] = 0;
    }

    unsigned char* c = v.getEntry(pos);
    bool isNull = (c == NULL);
    BOOST_REQUIRE(isNull);
    v.setEntry(pos, 10);
    c = v.getEntry(pos);
    isNull = (c == NULL);
    BOOST_REQUIRE(!isNull);
    BOOST_REQUIRE_EQUAL(*c, 10);

    VirtualRobot::VoxelTreeNDElement<unsigned char, N>* e = v.getLeafElement(pos);
    BOOST_REQUIRE(e != NULL);
    BOOST_REQUIRE(*(e->getEntry()) == 10);

    for (int i = 0; i < N; i++)
    {
        BOOST_CHECK_GE(e->getExtend(i), 0.5f * extend);
        BOOST_CHECK_LE(e->getExtend(i), 2.0f * extend);
    }
}


BOOST_AUTO_TEST_CASE(VoxelTreeNDSaveLoad)
{
    const unsigned int N = 6;
    float minB[N];
    float maxB[N];
    float discr[N];
    float extend = 5.0f;

    for (int i = 0; i < N; i++)
    {
        minB[i] = -100.0f;
        maxB[i] = 100.0f;
        discr[i] = extend;
    }

    VirtualRobot::VoxelTreeND<unsigned char, N> v(minB, maxB, discr, true);
    float pos[N];

    int TEST_LOOPS = 1000;

    for (int i = 0; i < TEST_LOOPS; i++)
    {
        for (int j = 0; j < N; j++)
        {
            pos[j] = float(rand() % 10000) / 10000.0f * 200.0f - 100.0f;
        }

        v.setEntry(pos, rand() % 255);
    }

    for (int i = 0; i < N; i++)
    {
        pos[i] = 17.0f;
    }

    v.setEntry(pos, 10);

    v.save("testVoxelTree.bin");
    VirtualRobot::VoxelTreeND<unsigned char, N>* v2 = VirtualRobot::VoxelTreeND<unsigned char, N>::load("testVoxelTree.bin");
    unsigned char* c = v2->getEntry(pos);
    bool isNull = (c == NULL);
    BOOST_REQUIRE(!isNull);
    BOOST_REQUIRE_EQUAL(*c, 10);

    delete v2;
}

BOOST_AUTO_TEST_CASE(VoxelTreeNDIterator)
{
    const unsigned int N = 6;
    float minB[N];
    float maxB[N];
    float discr[N];
    float extend = 10.0f;

    for (int i = 0; i < N; i++)
    {
        minB[i] = -100.0f;
        maxB[i] = 100.0f;
        discr[i] = extend;
    }

    VirtualRobot::VoxelTreeND<unsigned char, N> v(minB, maxB, discr, true);

    const int TEST_LOOPS = 10000;
    float pos[TEST_LOOPS][N];

    for (int i = 0; i < TEST_LOOPS; i++)
    {
        for (int j = 0; j < N; j++)
        {
            pos[i][j] = float(rand() % 10000) / 10000.0f * 200.0f - 100.0f;
        }

        v.setEntry(pos[i], rand() % 255);
    }

    VirtualRobot::VoxelTreeND<unsigned char, N>::ElementIterator it;
    VirtualRobot::VoxelTreeNDElement<unsigned char, N>* e = it.init(&v);
    int nrElements = 0;

    while (e)
    {
        nrElements++;
        e = it.getNextElement();
    }

    BOOST_CHECK_EQUAL(nrElements, TEST_LOOPS);
}


BOOST_AUTO_TEST_CASE(VoxelTreeNDMem)
{
    const unsigned int N = 6;
    float minB[N];
    float maxB[N];
    float discr[N];
    float extend = 10.0f;

    for (int i = 0; i < N; i++)
    {
        minB[i] = -100.0f;
        maxB[i] = 100.0f;
        discr[i] = extend;
    }

    VirtualRobot::VoxelTreeND<unsigned char, N> v(minB, maxB, discr, true);

    float pos[N];
    for (int j = 0; j < N; j++)
    {
        pos[j] = float(rand() % 10000) / 10000.0f * 200.0f - 100.0f;
    }

    v.setEntry(pos, rand() % 255);

    long structMem;
    long dataMem;
    v.getMemoryConsumtion(structMem,dataMem);
    long expectedStructMem = sizeof(VirtualRobot::VoxelTreeND<unsigned char, N>) + sizeof(VirtualRobot::VoxelTreeNDElement<unsigned char, N>) * v.getMaxLevels(); // the basic data structures
    expectedStructMem += (sizeof (VirtualRobot::VoxelTreeNDElement<unsigned char, N>*) * VirtualRobot::MathTools::pow_int(2, N)) * (N-1) ; // all except the leaf have to store an array of 64 pointers to the leafs
    BOOST_CHECK_EQUAL(structMem, expectedStructMem);
    BOOST_CHECK_EQUAL(dataMem, 1);
}


BOOST_AUTO_TEST_SUITE_END()
