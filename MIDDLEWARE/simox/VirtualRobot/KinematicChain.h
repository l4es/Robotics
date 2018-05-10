#ifndef _VirtualRobot_KinematicChain_h_
#define _VirtualRobot_KinematicChain_h_

#include "VirtualRobotImportExport.h"

#include "Nodes/RobotNode.h"
#include "RobotNodeSet.h"

#include <string>
#include <vector>


namespace VirtualRobot
{
    class Robot;

    /*!
        A KinematicChain is a RobotNodeSet that fulfills some constraints on the covered RobotNodes.
        The nodes form a kinematic chain, which means that two successive nodes are uniquely connected in the kinematic tree of the robot.
        E.g. node i and node i+1 are parent and child or node i+1 is child of a child of node i, etc.
        \see RobotNodeSet
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT KinematicChain : public RobotNodeSet
    {
    public:
        friend class RobotFactory;

        /*!
          Initialize this set with a vector of RobotNodes.
          The nodes have to be ordered: node[i] must be parent or grandparent or.. of node[i+1]
          \param name   The name of this kinematic chain.
          \param robotNodes A set of robot nodes.
          \param tcp    This specifies the TCP node of the robot's kinematic tree.
                        tcp does not have to be a node of this set. If not given, the last entry of robotNodes is used.
          \param kinematicRoot  This specifies the first node of the robot's kinematic tree to be used for updating all members of this set.
                                kinematicRoot does not have to be a node of this set. If not given, the first entry of robotNodes is used.
          */
        KinematicChain(const std::string& name, RobotPtr robot, const std::vector< RobotNodePtr >& robotNodes, RobotNodePtr tcp = RobotNodePtr(), RobotNodePtr kinematicRoot = RobotNodePtr());

        /*!
        */
        virtual ~KinematicChain();


    private:

    protected:
        RobotNodePtr kinematicRoot;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_KinematicChain_h_
