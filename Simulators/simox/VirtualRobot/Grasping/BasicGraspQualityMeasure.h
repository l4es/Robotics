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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef __BASIC_GRASP_QUALTIY_MEASURE_H__
#define __BASIC_GRASP_QUALTIY_MEASURE_H__

#include "../VirtualRobot.h"
#include "../EndEffector/EndEffector.h"
#include "../MathTools.h"
#include <vector>
#include <Eigen/Core>

namespace VirtualRobot
{
    /*!
        \brief An interface class for grasp quality algorithms.
        A basic quality score, relying on the number of contacts, is served by this implementation

        @see GraspStudio::GraspQualityMeasureWrenchSpace
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT BasicGraspQualityMeasure
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        BasicGraspQualityMeasure(VirtualRobot::SceneObjectPtr object);

        // destructor
        virtual ~BasicGraspQualityMeasure();

        /*!
            setup contact information
            the contact points are normalized by subtracting the COM
            the contact normals are normalize to unit length
        */
        virtual void setContactPoints(const VirtualRobot::EndEffector::ContactInfoVector& contactPoints);
        virtual void setContactPoints(const std::vector<VirtualRobot::MathTools::ContactPoint>& contactPoints6d);

        /*!
            Returns calculated grasp quality
        */
        virtual float getGraspQuality();


        //! Compute the grasp quality for the given contact points
        virtual bool calculateGraspQuality();

        virtual Eigen::Vector3f getCoM();

        virtual VirtualRobot::MathTools::ContactPoint getContactPointsCenter();

        virtual void setVerbose(bool enable);

        //! Returns description of this object
        virtual std::string getName();

        VirtualRobot::SceneObjectPtr getObject();

        virtual bool isValid();

    protected:


        //Object relevant parameters
        Eigen::Vector3f centerOfModel;
        float objectLength;
        float graspQuality;

        int maxContacts;

        VirtualRobot::SceneObjectPtr object;

        // object properties
        std::vector<VirtualRobot::MathTools::ContactPoint> contactPoints;  // in MM
        std::vector<VirtualRobot::MathTools::ContactPoint> contactPointsM; // converted to M
        bool verbose;
    };

} // namespace

#endif /* __BASIC_GRASP_QUALTIY_MEASURE_H__ */
