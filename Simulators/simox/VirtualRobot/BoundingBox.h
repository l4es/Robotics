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
#ifndef _VirtualRobot_BoundingBox_h_
#define _VirtualRobot_BoundingBox_h_

#include "VirtualRobotImportExport.h"
#include "MathTools.h"
#include "CollisionDetection/CollisionChecker.h"

#include <Eigen/Core>
#include <vector>

namespace VirtualRobot
{

    /*!
        An axis oriented bounding box.
        Todo: Some parts of this class are similar to MathTools::OOBB.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT BoundingBox
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        friend class CollisionChecker;

        BoundingBox();
        BoundingBox(const std::vector< Eigen::Vector3f >& p);

        /*!
            Returns true, if plane "hits" this bounding box.
        */
        bool planeGoesThrough(const VirtualRobot::MathTools::Plane& p);


        /*!
            Returns 8 points that define the bounding box
        */
        std::vector <Eigen::Vector3f> getPoints() const;

        //! Print some info
        void print();

        /*!
            Consider these points for min/max calculation
        */
        void addPoints(const std::vector < Eigen::Vector3f >& p);

        /*!
            Consider these points for min/max calculation
        */
        void addPoints(const BoundingBox& bbox);

        /*!
            Consider this point for min/max calculation
        */
        void addPoint(const Eigen::Vector3f& p);

        //! The axis oriented minimum value
        Eigen::Vector3f getMin() const;

        //! The axis oriented maximum value
        Eigen::Vector3f getMax() const;

        //! set min/max to zero.
        void clear();

        /*!
            Applies transformation to this bbox. Reorders min and max values according to pose.
        */
        void transform(Eigen::Matrix4f& pose);

        void scale(Eigen::Vector3f& scaleFactor);

    protected:
        Eigen::Vector3f min;
        Eigen::Vector3f max;
    };

} // namespace VirtualRobot

#endif /* _VirtualRobot_BoundingBox_h_ */
