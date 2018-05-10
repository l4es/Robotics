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
* @author     Markus Przybylski
* @copyright  2013 H2T,KIT
*             GNU Lesser General Public License
*
*/
#ifndef STROUTHELPERS_H
#define STROUTHELPERS_H

#include "../../GraspStudio.h"
#include <vector>
#include <Eigen/Geometry>
#include <sstream>
#include <string>
#include "VirtualRobot/VirtualRobot.h"
namespace GraspStudio
{

    class GRASPSTUDIO_IMPORT_EXPORT StrOutHelpers
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        StrOutHelpers();

        static std::string toString(std::vector<Eigen::Vector3f> v, bool lineBreak = false);
        static std::string toString(std::vector<Eigen::Vector3i> v, bool lineBreak = false);

        static std::string toString(Eigen::Vector3f v);
        static std::string toString(Eigen::Vector3i v);

    };
}
#endif // STROUTHELPERS_H
