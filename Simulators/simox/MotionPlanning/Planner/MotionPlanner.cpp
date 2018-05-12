
#include "MotionPlanner.h"

namespace Saba
{

    MotionPlanner::MotionPlanner(CSpacePtr cspace)
    {
        this->cspace = cspace;

        if (!cspace)
        {
            THROW_SABA_EXCEPTION("NULL data in MotionPlanner constructor");
        }

        dimension = cspace->getDimension();

        if (dimension == 0)
        {
            THROW_SABA_EXCEPTION("Need at least one dimensional cspace");
        }


        planningTime = 0.0f;

        stopSearch = false;
        maxCycles = 50000;              // stop if cycles are exceeded
        cycles = 0;

        startValid = false;
        goalValid = false;

        name = "Motion Planner";
    }

    MotionPlanner::~MotionPlanner()
    {
        reset();
    }

    void MotionPlanner::setName(std::string sName)
    {
        this->name = sName;
    }

    std::string MotionPlanner::getName()
    {
        return name;
    }


    void MotionPlanner::printConfig(bool printOnlyParams)
    {
        if (!printOnlyParams)
        {
            std::cout << "-- MotionPlanner config --" << std::endl;
            std::cout << "-------------------------------" << std::endl;
        }

        std::cout << "-- dimension: " << dimension << std::endl;
        std::cout << "-- maxCycles: " << maxCycles << std::endl;

        std::cout << "-- Start node: ";

        if (cspace)
        {
            cspace->printConfig(startConfig);
        }
        else
        {
            std::cout << " NOT SET";
        }

        std::cout << std::endl;
        std::cout << "-- Goal node: ";

        if (cspace)
        {
            cspace->printConfig(goalConfig);
        }
        else
        {
            std::cout << " NOT SET";
        }

        std::cout << std::endl;

        if (!printOnlyParams)
        {
            std::cout << "-------------------------------" << std::endl;
        }
    }




    bool MotionPlanner::setStart(const Eigen::VectorXf& c)
    {
        this->startConfig = c;
        startValid = false;

        if (cspace && !cspace->isCollisionFree(startConfig))
        {
            //if (startNode->obstacleDistance<=0)
            SABA_WARNING << " start pos is not collision free..." << std::endl;
            std::cout << "config:" << std::endl;
            cspace->printConfig(startConfig);
            std::cout << std::endl;
            return false;
        }

        if (cspace && !cspace->isInBoundary(startConfig))
        {
            SABA_WARNING << " start node does violate CSpace boundaries! " << std::endl;
            std::cout << "config:" << std::endl;
            cspace->printConfig(startConfig);
            std::cout << std::endl;
            return false;
        }

        startValid = true;
        return true;
    }


    bool MotionPlanner::setGoal(const Eigen::VectorXf& c)
    {
        this->goalConfig = c;
        goalValid = false;

        //tree->calculateObstacleDistance(goalNode);
        if (cspace && !cspace->isCollisionFree(goalConfig))
        {
            SABA_WARNING << " goal pos is not collision free..." << std::endl;
            std::cout << "config:" << std::endl;
            cspace->printConfig(goalConfig);
            std::cout << std::endl;
            return false;
        }

        if (cspace && !cspace->isInBoundary(goalConfig))
        {
            SABA_WARNING << " goal pos does violate CSpace boundaries!" << std::endl;
            std::cout << "config:" << std::endl;
            cspace->printConfig(goalConfig);
            std::cout << std::endl;
            return false;
        }

        goalValid = true;
        return true;
    }



    bool MotionPlanner::isInitialized()
    {
        if (!startValid)
        {
            return false;
        }

        if (!goalValid)
        {
            return false;
        }

        return true;
    }

    void MotionPlanner::setMaxCycles(unsigned int mc)
    {
        maxCycles = mc;
    }


    CSpacePathPtr MotionPlanner::getSolution()
    {
        if (!solution)
        {
            createSolution();
        }

        return solution;
    }

    void MotionPlanner::reset()
    {
        startConfig.setZero(1);
        goalConfig.setZero(1);
        startValid = false;
        goalValid = false;

        if (cspace)
        {
            cspace->reset();
        }

        planningTime = 0;
        cycles = 0;
    }

} // namespace
