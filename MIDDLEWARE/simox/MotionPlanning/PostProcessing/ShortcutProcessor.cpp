
#include "ShortcutProcessor.h"
#include "MotionPlanning/CSpace/CSpaceSampled.h"
#include "MotionPlanning/CSpace/CSpacePath.h"
#include <vector>
#include <time.h>
#include <math.h>

namespace Saba
{

    ShortcutProcessor::ShortcutProcessor(CSpacePathPtr path, CSpaceSampledPtr cspace, bool verbose) : PathProcessor(path, verbose), cspace(cspace)
    {
        stopOptimization = false;
    }

    ShortcutProcessor::~ShortcutProcessor()
    {
    }



    int ShortcutProcessor::tryRandomShortcut(int maxSolutionPathDist)
    {
        if (!path || !cspace)
        {
            SABA_ERROR << "NULL data" << endl;
            return 0;
        }

        if (verbose)
        {
            SABA_INFO << "Path length: " << optimizedPath->getLength() << std::endl;
            SABA_INFO << "Path nr of nodes: " << optimizedPath->getNrOfPoints() << std::endl;
        }

        int startNodeIndex, endNodeIndex;

        if (!selectCandidatesRandom(startNodeIndex, endNodeIndex, maxSolutionPathDist))
        {
            return 0;
        }

        if (verbose)
        {
            SABA_INFO << "-- start: " << startNodeIndex << ", end: " << endNodeIndex << std::endl;
        }

        if (!validShortcut(startNodeIndex, endNodeIndex))
        {
            return 0;
        }

        if (verbose)
        {
            std::cout << "Creating direct shortcut from node " << startNodeIndex << " to node " << endNodeIndex << std::endl;
        }

        // complete path valid and dist is shorter
        return doShortcut(startNodeIndex, endNodeIndex);
    }

    int ShortcutProcessor::doShortcut(int startIndex, int endIndex)
    {
        if (!optimizedPath)
            if (!initSolution())
            {
                return 0;
            }

        if (!optimizedPath || !cspace || startIndex < 0 || endIndex < 0 || startIndex >= (int)optimizedPath->getNrOfPoints() || endIndex >= (int)optimizedPath->getNrOfPoints())
        {
            return 0;
        }

        for (int i = endIndex - 1; i >= startIndex + 1; i--)
        {
            // erase solution positions
            optimizedPath->erasePosition(i);
        }

        if (verbose)
        {
            float distPathtest = optimizedPath->getLength(startIndex, startIndex + 1);
            std::cout << "-- erased intermediate positions, distPath startIndex to (startIndex+1): " << distPathtest << std::endl;
        }

        /*cout << "all2:" << endl;
        optimizedPath->print();*/

        Eigen::VectorXf s = optimizedPath->getPoint(startIndex);
        Eigen::VectorXf e = optimizedPath->getPoint(startIndex + 1);

        // create intermediate path
        CSpacePathPtr intermediatePath = cspace->createPath(s, e);
        int newP = 0;

        if (intermediatePath->getNrOfPoints() > 2)
        {
            newP = intermediatePath->getNrOfPoints() - 2;
            /*cout << "before:" << endl;
            optimizedPath->print();
            cout << "interm path:" << endl;
            intermediatePath->print();*/
            intermediatePath->erasePosition(intermediatePath->getNrOfPoints() - 1);
            intermediatePath->erasePosition(0);
            /*cout << "interm path without start end:" << endl;
            intermediatePath->print();*/
            optimizedPath->insertTrajectory(startIndex + 1, intermediatePath);
            /*cout << "after:" << endl;
            optimizedPath->print(); */
        }

        if (verbose)
        {
            float sum = 0.0f;

            for (int u = startIndex; u <= startIndex + newP; u++)
            {
                float distPathtest2 = optimizedPath->getLength(u, u + 1);
                sum += distPathtest2;
                std::cout << "---- intermediate position: " << u << ", distPath to next pos: " << distPathtest2 << ", sum:" << sum << std::endl;
            }
        }

        int nodes = endIndex - startIndex - 1 + newP;

        if (verbose)
        {
            std::cout << "-- end, nodes: " << nodes << std::endl;
        }


        return nodes;
    }


    CSpacePathPtr ShortcutProcessor::optimize(int optimizeSteps)
    {
        return shortenSolutionRandom(optimizeSteps);
    }

    CSpacePathPtr ShortcutProcessor::shortenSolutionRandom(int shortenLoops /*=300*/, int maxSolutionPathDist)
    {
        stopOptimization = false;
        THROW_VR_EXCEPTION_IF((!cspace || !path), "NULL data");
        THROW_VR_EXCEPTION_IF(!initSolution(), "Could not init...");
        int counter = 0;

        if (optimizedPath->getNrOfPoints() <= 2)
        {
            return optimizedPath;
        }

        int result = 0;
        int beforeCount = (int)optimizedPath->getNrOfPoints();
        float beforeLength = optimizedPath->getLength();

        if (verbose)
        {
            SABA_INFO << ": solution size before shortenSolutionRandom:" << beforeCount << std::endl;
            SABA_INFO << ": solution length before shortenSolutionRandom:" << beforeLength << std::endl;
        }

        clock_t startT = clock();

        int red;
        int loopsOverall = 0;

        while (counter < shortenLoops && !stopOptimization)
        {
            loopsOverall++;
            red = tryRandomShortcut(maxSolutionPathDist);

            counter++;
            result += red;
        }

        if (stopOptimization)
        {
            SABA_INFO << "optimization was stopped" << std::endl;
        }

        int afterCount = (int)optimizedPath->getNrOfPoints();
        float afterLength = optimizedPath->getLength();
        clock_t endT = clock();
        float timems = (float)(endT - startT) / (float)CLOCKS_PER_SEC * 1000.0f;

        if (verbose)
        {
            SABA_INFO << ": shorten loops: " << loopsOverall << std::endl;
            SABA_INFO << ": shorten time: " << timems << " ms " << std::endl;
            SABA_INFO << ": solution size after ShortenSolutionRandom (nr of positions) : " << afterCount << std::endl;
            SABA_INFO << ": solution length after ShortenSolutionRandom : " << afterLength << std::endl;
        }

        return optimizedPath;
    }

    void ShortcutProcessor::doPathPruning()
    {
        THROW_VR_EXCEPTION_IF(!initSolution(), "Could not init");

        unsigned int i = 0;

        while (i < optimizedPath->getNrOfPoints() - 2)
        {
            Eigen::VectorXf startConfig = optimizedPath->getPoint(i);
            Eigen::VectorXf endConfig = optimizedPath->getPoint(i + 2);

            if (cspace->isPathValid(startConfig, endConfig))
            {
                optimizedPath->erasePosition(i + 1);

                if (i > 0)
                {
                    i--;
                }
            }
            else
            {
                i++;
            }
        }
    }

    bool ShortcutProcessor::selectCandidatesRandom(int& storeStartIndex, int& storeEndIndex, int maxSolutionPathDist)
    {
        if (!optimizedPath)
            if (!initSolution())
            {
                return false;
            }

        if (!optimizedPath || optimizedPath->getNrOfPoints() <= 2)
        {
            return false;
        }

        if (maxSolutionPathDist < 2)
        {
            maxSolutionPathDist = 2;
        }

        storeStartIndex = (int)(rand() % (optimizedPath->getNrOfPoints() - 2));

        int remainig = optimizedPath->getNrOfPoints() - 2 - storeStartIndex;

        if (remainig > maxSolutionPathDist)
        {
            remainig = maxSolutionPathDist;
        }

        if (remainig <= 0)
        {
            return false;
        }

        int dist = 2 + rand() % (remainig);
        storeEndIndex = storeStartIndex + dist;

        if (storeStartIndex < 0 || storeEndIndex < 0)
        {
            return false;
        }

        if (storeEndIndex > (int)optimizedPath->getNrOfPoints() - 1)
        {
            return false;
        }

        if (storeStartIndex > (int)optimizedPath->getNrOfPoints() - 1)
        {
            return false;
        }

        if ((storeEndIndex - storeStartIndex) <= 1)
        {
            return false;
        }

        return true;
    }

    bool ShortcutProcessor::validShortcut(int startIndex, int endIndex)
    {
        if (!optimizedPath)
            if (!initSolution())
            {
                return false;
            }

        Eigen::VectorXf s = optimizedPath->getPoint(startIndex);
        Eigen::VectorXf e = optimizedPath->getPoint(endIndex);
        Eigen::VectorXf d = e - s;

        // test line between start and end
        float distShortcut = d.norm();
        float distPath = optimizedPath->getLength(startIndex, endIndex);

        // -------------------------------------------------------------------
        // DEBUG
        if (verbose)
        {
            std::cout << "-- distShortcut: " << distShortcut << " distPath: " << distPath << std::endl;
        }

        // -------------------------------------------------------------------

        if (distShortcut >= distPath * 0.99f)
        {
            if (verbose)
            {
                cout << "Path is not shorter..." << endl;
            }

            return false;
        }

        // -------------------------------------------------------------------
        // DEBUG
        if (verbose)
        {
            std::cout << ": Shortcut Path shorter!" << std::endl;
        }

        // -------------------------------------------------------------------

        return cspace->isPathValid(s, e);
    }

    bool ShortcutProcessor::initSolution()
    {
        if (!path)
        {
            VR_ERROR << "Wrong parameters or no path to smooth..." << std::endl;
            return false;
        }

        optimizedPath = path->clone();

        if (!optimizedPath)
        {
            VR_ERROR << "Wrong parameters or no path to smooth..." << std::endl;
            return false;
        }

        return true;
    }


} // namespace
