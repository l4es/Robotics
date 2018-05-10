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
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef __PoseQualityMeasurement_H_
#define __PoseQualityMeasurement_H_

#include "../VirtualRobotImportExport.h"
#include "../Robot.h"
#include "../VirtualRobotException.h"
#include "../Nodes/RobotNode.h"
#include "../RobotNodeSet.h"
#include "../IK/DifferentialIK.h"

#include <string.h>
#include <vector>
#include <Eigen/Core>

namespace VirtualRobot
{
    /**
    * \class PoseQualityMeasurement
    *
    * An interface definition for quality measurements of poses.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT PoseQualityMeasurement
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PoseQualityMeasurement(VirtualRobot::RobotNodeSetPtr rns);
        ~PoseQualityMeasurement();

        /*!
            The main method for determining the pose quality.
            The current configuration of the corresponding RNS is analyzed and the quality is returned.
            See derived classes for details.
        */
        virtual float getPoseQuality();

        /*!
            The quality is determined for a given Cartesian direction.
            The current configuration of the corresponding RNS is analyzed and the quality is returned.
            See derived classes for details.
            \param direction A 3d or 6d vector with the Cartesian direction to investigate.
        */
        virtual float getPoseQuality(const Eigen::VectorXf& direction);


        void setVerbose(bool v);

        /*!
            Returns the RobotNodeSte that is used for computing the manipulability.
        */
        VirtualRobot::RobotNodeSetPtr getRNS();

        //! A string that identifies the type of pose quality measure.
        std::string getName();

        //! Indicates if joint limits are considered.
        virtual bool consideringJointLimits();

        /*!
            Consider obstacles. Here, the shortest distance on the surface of the RNS to an obstacle is set (in TCP coords).
            This obstacle vector may be considered by any further calculations (depending on the implementation).
        */
        virtual void setObstacleDistanceVector(const Eigen::Vector3f& directionSurfaceToObstance);
        virtual void disableObstacleDistance();

    protected:
        std::string name;
        VirtualRobot::RobotNodeSetPtr rns;

        bool considerObstacle;
        Eigen::Vector3f obstacleDir;

        bool verbose;
    };

}

#endif // __PoseQualityMeasurement_H_
