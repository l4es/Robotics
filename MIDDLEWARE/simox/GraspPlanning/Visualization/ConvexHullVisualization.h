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
* @package    GraspStudio
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _GraspStudio_ConvexHullVisualization_h_
#define _GraspStudio_ConvexHullVisualization_h_

#include "../GraspStudio.h"
#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/MathTools.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace GraspStudio
{
    /*!
     *
     * \brief A visualization of a convex hull
     * @see CoinConvexHullVisualization
     *
     */
    class GRASPSTUDIO_IMPORT_EXPORT ConvexHullVisualization
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
        */
        ConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr convHull, bool useFirst3Coords = true);
        ConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr convHull);

        /*!
        */
        virtual ~ConvexHullVisualization();

    protected:

        VirtualRobot::MathTools::ConvexHull3DPtr convHull3D;
        VirtualRobot::MathTools::ConvexHull6DPtr convHull6D;
        bool useFirst3Coords;

    };

} // namespace GraspStudio

#endif // _GraspStudio_ConvexHullVisualization_h_
