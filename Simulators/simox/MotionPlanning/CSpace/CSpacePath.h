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
#ifndef _saba_CSpacePath_h
#define _saba_CSpacePath_h

#include "../Saba.h"
#include <vector>
#include <string>

#include <Eigen/StdVector>

#include "CSpace.h"

#include <VirtualRobot/Trajectory.h>

namespace Saba
{

    /*!
     *
     * \brief A path in c-space.
     *
     */
    class SABA_IMPORT_EXPORT CSpacePath : public VirtualRobot::Trajectory
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor. Dimension of this path is retrieved from cspace.
            \param cspace Is used to retrieve the dimension of the corresponding c-space and the corresponding joints
            \param name An optional string, identifying this path.
        */
        CSpacePath(CSpacePtr cspace, const std::string& name = "");

        //! Destructor
        virtual ~CSpacePath();



        /*!
          Creates a copy of the path instance.
          \return pointer to new instance (copy)
        */
        CSpacePathPtr clone() const;

        CSpacePathPtr createSubPath(unsigned int startIndex, unsigned int endIndex) const;

        /*!
            Return euclidean c-space length of complete path.
            Any weights that may be defined in the corresponding c-space are considered.
        */
        virtual float getLength() const;

        /*!
            Return euclidean c-space length of complete path
            \param useCSpaceWeights When set, the weight that might have been specified in the corresponding c-space
            are used for computing the path length.
        */
        float getLength(bool useCSpaceWeights) const;

        /*!
            Return length of part of the path (in CSpace!)
            \param startIndex The start point
            \param endIndex The end point
            \param useCSpaceWeights When set, the weights that have been specified in the corresponding c-space are used for computing the path length.
        */
        float getLength(unsigned int startIndex, unsigned int endIndex, bool useCSpaceWeights = true) const;


        /*!
            return position on path for time t (0<=t<=1)
            If storeIndex!=NULL the index of the last path point is stored.
            This method consideres weighting of c-space dimensions!
         */
        virtual void interpolate(float t, Eigen::VectorXf& storePos, int* storeIndex = NULL) const;

        //! return time t (0<=t<=1) for path entry with number nr
        virtual float getTime(unsigned int nr);


        CSpacePtr getCSpace();
        std::vector<Eigen::Matrix4f > createWorkspacePath(VirtualRobot::RobotNodePtr r = VirtualRobot::RobotNodePtr());

    protected:
        CSpacePtr cspace;
    };

} // namespace Saba


#endif // _saba_CSpacePath_h
