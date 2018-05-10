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
#ifndef __CONTACT_CONE_GENARTOR_H__
#define __CONTACT_CONE_GENARTOR_H__

#include "GraspStudio.h"
#include <VirtualRobot/MathTools.h>
#include <vector>
#include <Eigen/Core>

namespace GraspStudio
{
    /*!
        \brief Creates approximated representations of contact cones.
    */
    class GRASPSTUDIO_IMPORT_EXPORT ContactConeGenerator
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        //! constructor
        ContactConeGenerator(int coneSamples = 8, float frictionCoeff = 0.25f, float unitForce = 1.0f);

        //! destructor
        ~ContactConeGenerator();

        /*!
            Computes the cone with normals. coneSamples computed points are appended to storeConePoints.
        */
        void computeConePoints(const VirtualRobot::MathTools::ContactPoint& point, std::vector<VirtualRobot::MathTools::ContactPoint>& storeConePoints);

        //! Computes the cone points without normals. coneSamples computed points are appended to storeConePoints.
        void computeConePoints(const VirtualRobot::MathTools::ContactPoint& point, std::vector<Eigen::Vector3f>& storeConePoints);

        /*!
            Returns the opening angle of a friction cone. [rad]
        */
        float getConeAngle();

        /*!
            Returns the radius a friction cone.
        */
        float getConeRadius();

        /*!
            Returns the height a friction cone.
        */
        float getConeHeight();

    private:

        //Friction cone relevant parameters
        double unitForce;
        double frictionCoeff;
        double frictionConeAngle;
        double frictionConeRad;
        double frictionConeHeight;
        std::vector< Eigen::Vector3f > frictionConeRimPoints;
        int frictionConeSamples;

    };
}

#endif /* __CONTACT_CONE_GENARTOR_H__ */
