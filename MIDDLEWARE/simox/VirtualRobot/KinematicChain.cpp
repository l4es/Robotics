
#include "KinematicChain.h"
#include "Robot.h"
#include "VirtualRobotException.h"


#include <algorithm>

namespace VirtualRobot
{

    KinematicChain::KinematicChain(const std::string& name, RobotPtr robot, const std::vector< RobotNodePtr >& robotNodes, RobotNodePtr tcp, RobotNodePtr kinematicRoot /*= RobotNodePtr()*/)
        : RobotNodeSet(name, robot, robotNodes, kinematicRoot)
    {
        if (robotNodes.size() == 0)
        {
            THROW_VR_EXCEPTION("Zero sized Kinematic chain is not allowed");
        }

        for (unsigned int i = 0; i < this->robotNodes.size() - 1; i++)
        {
            if (!this->robotNodes[i]->hasChild(this->robotNodes[i + 1], true))
            {
                THROW_VR_EXCEPTION("Robot node <" << this->robotNodes[i + 1]->getName() << "> is not a child of robot node <" << this->robotNodes[i]->getName() << ">. This is not allowed in KinematicChains...");
            }

            if (robotNodes[i]->getRobot() != robotNodes[i + 1]->getRobot())
            {
                THROW_VR_EXCEPTION("Robot node <" << this->robotNodes[i + 1]->getName() << "> his linked to different robot than robot node <" << this->robotNodes[i]->getName() << ">. This is not allowed in KinematicChains...");
            }

        }

        this->robot = robotNodes[0]->getRobot();

        if (!tcp)
        {
            this->tcp = robotNodes[this->getSize() - 1];
        }
        else
        {
            this->tcp = tcp;
        }
    }


    KinematicChain::~KinematicChain()
    {

    }



} // namespace VirtualRobot
