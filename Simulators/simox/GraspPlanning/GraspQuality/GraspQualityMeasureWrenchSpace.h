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
#ifndef __GRASP_QUALTIY_MEASURE_WRENCH_H__
#define __GRASP_QUALTIY_MEASURE_WRENCH_H__

#include "../GraspStudio.h"
#include "GraspQualityMeasure.h"

#include <Eigen/Core>

namespace GraspStudio
{

    /*!
        \brief An efficient implementation of the grasp wrench space algorithm for grasp quality evaluation.

        the grasp wrench space algorithm is widely used in the context of grasp planning. By analyzing
        the grasp wrench space (GWS) of a given set of contact points, a quality score of a grasp
        can be evaluated.
        In this implementation, additionally an object specific wrench space (WS) is calculated, which
        approximatevly represents a "perfect" grasp. This object is used to normalize the quality score.
    */
    class GRASPSTUDIO_IMPORT_EXPORT GraspQualityMeasureWrenchSpace : public GraspQualityMeasure
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        GraspQualityMeasureWrenchSpace(VirtualRobot::SceneObjectPtr object, float unitForce = 1.0f, float frictionConeCoeff = 0.35f, int frictionConeSamples = 8);
        // destructor
        ~GraspQualityMeasureWrenchSpace();


        /*!
            The OWS should not change for an object, so if you have the data calculated once, you can set them here
            (The values for the OWS can change slightly, since the facets used for generation of the OWS are sampled randomly,
            in case there are more than 400 of them)
        */
        virtual void preCalculatedOWS(float minDist, float volume);

        /*!
            Returns f_max_gws / f_max_OWS
            with f_max_gws = max distance of GWS hull center to one of its facets
            with f_max_ows = max distance of OWS hull center to one of its facets
            -> also known as "epsilon" quality == radius of larges enclosing 6D ball
        */
        virtual float getGraspQuality();

        /*!
            Volume grasp quality ratio of GWS volume / OWS volume
            -> also known as "v" quality
        */
        virtual float getVolumeGraspMeasure();

        /*
            Checks if wrench space origin is inside GWS-Hull
        */
        virtual bool isGraspForceClosure();

        /*
            Returns the internally calculated convex hull object (ObjectWrenchSpace)
            This hull depends on the object
        */
        virtual VirtualRobot::MathTools::ConvexHull6DPtr getConvexHullOWS()
        {
            return convexHullOWS;
        }
        /*
            Returns the internally calculated convex hull object (GraspWrenchSpace)
            This hull depends on the contacts
        */
        virtual VirtualRobot::MathTools::ConvexHull6DPtr getConvexHullGWS()
        {
            return convexHullGWS;
        }

        void calculateOWS(int samplePoints = 300);
        void calculateGWS();
        bool OWSExists()
        {
            return OWSCalculated;
        }
        bool GWSExists()
        {
            return GWSCalculated;
        }

        VirtualRobot::MathTools::ContactPoint getCenterGWS()
        {
            return convexHullCenterGWS;
        }
        VirtualRobot::MathTools::ContactPoint getCenterOWS()
        {
            return convexHullCenterOWS;
        }

        /*!
        setup contact information
        the contact points are normalized by subtracting the COM
        the contact normals are normalize to unit length
        */
        virtual void setContactPoints(const std::vector<VirtualRobot::MathTools::ContactPoint>& contactPoints);
        virtual void setContactPoints(const VirtualRobot::EndEffector::ContactInfoVector& contactPoints);

        virtual bool calculateGraspQuality();
        virtual bool calculateObjectProperties();

        //! Returns description of this object
        virtual std::string getName();

        virtual float getOWSMinOffset()
        {
            return minOffsetOWS;
        }
        virtual float getOWSVolume()
        {
            return volumeOWS;
        }

        static std::vector<VirtualRobot::MathTools::ContactPoint> createWrenchPoints(std::vector<VirtualRobot::MathTools::ContactPoint>& points, const Eigen::Vector3f& centerOfModel, float objectLengthMM);

        //! Goes through all facets of convex hull and searches the minimum distance to it's center
        static float minOffset(VirtualRobot::MathTools::ConvexHull6DPtr ch);

    protected:

        //Methods
        VirtualRobot::MathTools::ConvexHull6DPtr calculateConvexHull(std::vector<VirtualRobot::MathTools::ContactPoint>& points);
        VirtualRobot::MathTools::ContactPoint calculateHullCenter(VirtualRobot::MathTools::ConvexHull6DPtr hull);

        float minDistanceToGWSHull(VirtualRobot::MathTools::ContactPoint& point);


        bool isOriginInGWSHull();
        void printContacts(std::vector<VirtualRobot::MathTools::ContactPoint>& points);
        static Eigen::Vector3f crossProductPosNormalInv(const VirtualRobot::MathTools::ContactPoint& v1);

        //For safety
        bool OWSCalculated;
        bool GWSCalculated;

        //For Object and Grasp Wrench Space Calculation
        VirtualRobot::MathTools::ConvexHull6DPtr convexHullOWS;
        VirtualRobot::MathTools::ConvexHull6DPtr convexHullGWS;
        VirtualRobot::MathTools::ContactPoint convexHullCenterGWS;
        VirtualRobot::MathTools::ContactPoint convexHullCenterOWS;

        // the minimal distance from center of OWS to one of it's facets
        float minOffsetOWS;
        float volumeOWS;
    };

} // namespace

#endif /* __GRASP_QUALTIY_MEASURE_WRENCH_H__ */
