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
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _Saba_CSpaceSampled_h
#define _Saba_CSpaceSampled_h

#include "../Saba.h"
#include "CSpace.h"
#include <string>



namespace Saba
{

    /*!

      \brief This class represents a sampled-based configuration space. This is the main class for RRT-related planning.

     A CSpace is defined by a set of robot nodes and a collision manager.
     The RobotNodeSet specifies the dimension and the borders of the CSpace.
     The collision manager is used to check configurations for collisions. Here,
     multiple sets of objects can be defined which are internally mutually checked for
     collisions. This allows to specify complex collision queries.
     The sampling-based c-space relies on two parameters: The sampling size, which specifies
     the (c-space) distance between two succeeding configurations on a path segment. This
     parameter affects how many intermediate configurations are added when creating new paths.
     The second parameter is called DCD sampling size, where DCD stands for discrete collision detection.
     This parameter is needed for the check if a path segment is valid or not. Therefore intermediate
     configurations on the path segment are checked for collisions with the given maximum distance between two
     neighboring intermediate configuration. This parameter affects both, the performance (the Rrt approach spends
     most of the time for collision detection) and the reliability of the results. Since there is no guarantee that
     all potential configurations are detected with sampling-based approaches, a large DCD sampling parameter
     will increase the chance of missing a collision.

     Constraints can be considered by adding instances of ConfigurationConstraintPtr.

      @see CSpace
      @see Rrt
      @see BiRrt
      @see VirtualRobot::CDManager
     */
    class SABA_IMPORT_EXPORT CSpaceSampled : public CSpace
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Construct a c-space that represents the given set of joints. The dimension of this
            c-space is related to the number of joints in robotNodes.
            The boundaries of this c-space are set according to definitions in robotNodes.
        */
        CSpaceSampled(VirtualRobot::RobotPtr robot, VirtualRobot::CDManagerPtr collisionManager, VirtualRobot::RobotNodeSetPtr robotNodes, unsigned int maxConfigs = 50000, unsigned int randomSeed = 0);

        virtual ~CSpaceSampled();

        //! sets sampling step size (used when adding new paths in CSpace)
        void setSamplingSize(float fSize);
        //! sets sampling step size used for discrete collision checking
        void setSamplingSizeDCD(float fSize);


        float getSamplingSize()
        {
            return samplingSizePaths;
        };
        float getSamplingSizeDCD()
        {
            return samplingSizeDCD;
        };



        virtual CSpacePtr clone(VirtualRobot::CollisionCheckerPtr newColChecker, VirtualRobot::RobotPtr newRobot, VirtualRobot::CDManagerPtr newCDM, unsigned int newRandomSeed = 0);

        /*!
            Checks the middle configuration and when it is not invalid (in collision or constraints are violated) the path is split
            and both parts are recursively checked until the distance is smalled than the DCD sampling size.
            Recursion is performed maximal recursionMaxDepth times, since an array of temporary variables is used,
            in order to avoid slow allocating/deallocating of memory.
        */
        bool isPathValid(const Eigen::VectorXf& q1, const Eigen::VectorXf& q2);

        /*!
        Create a path from start to goal without any checks.
        Intermediate configurations are added according to the current implementation of the cspace.
        */
        virtual CSpacePathPtr createPath(const Eigen::VectorXf& start, const Eigen::VectorXf& goal);

        /*!
            Create a path from start to the goal configuration.
            In case an invalid (collision/constraints) position is detected the appending is stopped and the valid part of the path is returned.
            \param start The start
            \param goal The goal
            \param storeAddedLength The length of the valid path is stored here (1.0 means the complete path from start to goal was valid)
        */
        virtual CSpacePathPtr createPathUntilInvalid(const Eigen::VectorXf& start, const Eigen::VectorXf& goal, float& storeAddedLength);


    protected:

        float samplingSizePaths;                //!< euclidean sample size
        float samplingSizeDCD;                  //!< euclidean sample size for collision check
        Eigen::VectorXf checkPathConfig;

        int recursiveTmpValuesIndex;
        std::vector<Eigen::VectorXf> recursiveTmpValues;
        Eigen::VectorXf tmpConfig;

        const int recursionMaxDepth;
    };

}

#endif // _Saba_CSpaceSampled_h
