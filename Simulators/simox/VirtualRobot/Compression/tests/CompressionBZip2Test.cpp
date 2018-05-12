/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Nikolaus Vahrenkamp
*/

#define BOOST_TEST_MODULE VirtualRobot_CompressionBZip2Test

#include <VirtualRobot/VirtualRobotTest.h>
#include <VirtualRobot/Compression/CompressionBZip2.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <Eigen/Core>

#include <fstream>
#include <istream>
#include <time.h>

BOOST_AUTO_TEST_SUITE(Compression)

BOOST_AUTO_TEST_CASE(testInvalidCreation)
{
    std::istream* is = NULL;
    std::ostream* os = NULL;
    BOOST_CHECK_THROW(new VirtualRobot::CompressionBZip2(is), VirtualRobot::VirtualRobotException);
    BOOST_CHECK_THROW(new VirtualRobot::CompressionBZip2(os), VirtualRobot::VirtualRobotException);
}

#define BLOCK_SIZE_COMPRESSION_TEST 8000

BOOST_AUTO_TEST_CASE(testNullBlock)
{
    // COMPRESS
    std::stringstream ios;
    unsigned char blockN[BLOCK_SIZE_COMPRESSION_TEST];
    memset(blockN, 0, sizeof(unsigned char)*BLOCK_SIZE_COMPRESSION_TEST);
    VirtualRobot::CompressionBZip2* bzip2 = NULL;
    BOOST_CHECK_NO_THROW(bzip2 = new VirtualRobot::CompressionBZip2((std::ostream*)(&ios)));

    bool ok = false;
    BOOST_CHECK_NO_THROW(ok = bzip2->write((void*)(blockN), BLOCK_SIZE_COMPRESSION_TEST));
    BOOST_CHECK_EQUAL(ok, true);
    BOOST_CHECK_NO_THROW(ok = bzip2->close());
    BOOST_CHECK_EQUAL(ok, true);


    // UNCOMPRESS

    // set position to start
    ios.seekg(0);
    unsigned char blockN2[BLOCK_SIZE_COMPRESSION_TEST];
    memset(blockN2, 1, sizeof(unsigned char)*BLOCK_SIZE_COMPRESSION_TEST);
    VirtualRobot::CompressionBZip2* bzip2b = NULL;
    BOOST_CHECK_NO_THROW(bzip2b = new VirtualRobot::CompressionBZip2((std::istream*)(&ios)));

    ok = false;
    int nrBytes = 0;
    BOOST_CHECK_NO_THROW(ok = bzip2b->read((void*)(blockN2), BLOCK_SIZE_COMPRESSION_TEST, nrBytes));
    BOOST_CHECK_EQUAL(ok, true);
    BOOST_CHECK_EQUAL(nrBytes, BLOCK_SIZE_COMPRESSION_TEST);
    BOOST_CHECK_NO_THROW(ok = bzip2b->close());
    BOOST_CHECK_EQUAL(ok, true);

    BOOST_CHECK_EQUAL(ios.eof(), true);

    for (int i = 0; i < BLOCK_SIZE_COMPRESSION_TEST; i++)
    {
        BOOST_CHECK_EQUAL(blockN2[i], 0);
    }

    delete bzip2;
    delete bzip2b;

}

#define NR_BLOCKS_COMPRESSION_TEST 17

BOOST_AUTO_TEST_CASE(testMultipleRandomBlocks)
{
    srand((unsigned int)time(NULL));
    // COMPRESS
    std::stringstream ios;
    unsigned char blockN[NR_BLOCKS_COMPRESSION_TEST][BLOCK_SIZE_COMPRESSION_TEST];
    unsigned char blockN2[NR_BLOCKS_COMPRESSION_TEST][BLOCK_SIZE_COMPRESSION_TEST];

    for (int i = 0; i < NR_BLOCKS_COMPRESSION_TEST; i++)
    {
        for (int j = 0; j < BLOCK_SIZE_COMPRESSION_TEST; j++)
        {
            blockN[i][j] = rand() % 256;
            blockN2[i][j] = 0;
        }
    }

    VirtualRobot::CompressionBZip2* bzip2 = NULL;
    BOOST_CHECK_NO_THROW(bzip2 = new VirtualRobot::CompressionBZip2((std::ostream*)(&ios)));

    bool ok = false;

    for (int j = 0; j < NR_BLOCKS_COMPRESSION_TEST; j++)
    {
        BOOST_CHECK_NO_THROW(ok = bzip2->write((void*)(blockN[j]), BLOCK_SIZE_COMPRESSION_TEST));
        BOOST_CHECK_EQUAL(ok, true);
    }

    BOOST_CHECK_NO_THROW(ok = bzip2->close());
    BOOST_CHECK_EQUAL(ok, true);


    // UNCOMPRESS

    // set position to start
    ios.seekg(0);
    VirtualRobot::CompressionBZip2* bzip2b = NULL;
    BOOST_CHECK_NO_THROW(bzip2b = new VirtualRobot::CompressionBZip2((std::istream*)(&ios)));

    ok = false;
    int nrBytes = 0;

    for (int j = 0; j < NR_BLOCKS_COMPRESSION_TEST; j++)
    {
        BOOST_CHECK_NO_THROW(ok = bzip2b->read((void*)(blockN2[j]), BLOCK_SIZE_COMPRESSION_TEST, nrBytes));
        BOOST_CHECK_EQUAL(ok, true);
        BOOST_CHECK_EQUAL(nrBytes, BLOCK_SIZE_COMPRESSION_TEST);
    }

    BOOST_CHECK_NO_THROW(ok = bzip2b->close());
    BOOST_CHECK_EQUAL(ok, true);

    BOOST_CHECK_EQUAL(ios.eof(), true);

    // check data
    for (int i = 0; i < NR_BLOCKS_COMPRESSION_TEST; i++)
    {
        for (int j = 0; j < BLOCK_SIZE_COMPRESSION_TEST; j++)
        {
            BOOST_CHECK_EQUAL(blockN2[i][j], blockN[i][j]);
        }
    }

    delete bzip2;
    delete bzip2b;
}


BOOST_AUTO_TEST_CASE(testCorrectEnding)
{
    static const int sizeSmall = 23;
    // COMPRESS
    std::stringstream ios;
    unsigned char blockN[sizeSmall];
    memset(blockN, 0, sizeof(unsigned char)*sizeSmall);
    VirtualRobot::CompressionBZip2* bzip2 = NULL;
    BOOST_CHECK_NO_THROW(bzip2 = new VirtualRobot::CompressionBZip2((std::ostream*)(&ios)));

    bool ok = false;
    BOOST_CHECK_NO_THROW(ok = bzip2->write((void*)(blockN), sizeSmall));
    BOOST_CHECK_EQUAL(ok, true);
    BOOST_CHECK_NO_THROW(ok = bzip2->close());
    BOOST_CHECK_EQUAL(ok, true);

    ios.put('s');
    ios.put('i');
    ios.put('m');
    ios.put('o');
    ios.put('x');

    // UNCOMPRESS

    // set position to start
    ios.seekg(0);
    unsigned char blockN2[sizeSmall];
    memset(blockN2, 1, sizeof(unsigned char)*sizeSmall);
    VirtualRobot::CompressionBZip2* bzip2b = NULL;
    BOOST_CHECK_NO_THROW(bzip2b = new VirtualRobot::CompressionBZip2((std::istream*)(&ios)));

    ok = false;
    int nrBytes = 0;
    BOOST_CHECK_NO_THROW(ok = bzip2b->read((void*)(blockN2), sizeSmall, nrBytes));
    BOOST_CHECK_EQUAL(ok, true);
    BOOST_CHECK_EQUAL(nrBytes, sizeSmall);
    BOOST_CHECK_NO_THROW(ok = bzip2b->close());
    BOOST_CHECK_EQUAL(ok, true);


    for (int i = 0; i < sizeSmall; i++)
    {
        BOOST_CHECK_EQUAL(blockN2[i], 0);
    }

    BOOST_CHECK_EQUAL(ios.eof(), false);

    char s;
    ios.get(s);
    BOOST_CHECK_EQUAL(ios.eof(), false);
    BOOST_CHECK_EQUAL(s, 's');
    ios.get(s);
    BOOST_CHECK_EQUAL(ios.eof(), false);
    BOOST_CHECK_EQUAL(s, 'i');
    ios.get(s);
    BOOST_CHECK_EQUAL(ios.eof(), false);
    BOOST_CHECK_EQUAL(s, 'm');
    ios.get(s);
    BOOST_CHECK_EQUAL(ios.eof(), false);
    BOOST_CHECK_EQUAL(s, 'o');
    ios.get(s);
    BOOST_CHECK_EQUAL(ios.eof(), false);
    BOOST_CHECK_EQUAL(s, 'x');
    ios.get(s);
    BOOST_CHECK_EQUAL(ios.eof(), true);

    delete bzip2;
    delete bzip2b;

}


BOOST_AUTO_TEST_SUITE_END()
