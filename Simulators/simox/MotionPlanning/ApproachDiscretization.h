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
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/

#ifndef _SABA_APPROACH_DISCRETIZATION_H_
#define _SABA_APPROACH_DISCRETIZATION_H_

#include "Saba.h"

#include <vector>
#include <map>
#include <float.h>
#include <VirtualRobot/SphereApproximator.h>

#include "CSpace/CSpaceNode.h"

namespace Saba
{

    /*!
    * This discretization class can be used to encode approach directions.
    * Internally all approach directions are mapped onto the surface of a discretized sphere.
    * Each face of the PoseRelationSphere holds a list with corresponding RRT nodes,
    * lying in the direction that is covered by the triangle.
    * The method getGoodRatedNode can be used to retrieve RRT-nodes for which the approach directions are uniformly sampled.
    * (further details can be found in "Integrated Grasp and Motion Planning", ICRA 2010, Vahrenkamp et al.)
    *
    */

    class SABA_IMPORT_EXPORT ApproachDiscretization
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        ApproachDiscretization(float radius = 100.0f, int steps = 3);
        virtual ~ApproachDiscretization();

        Eigen::Matrix4f getGlobalPose() const;

        /*!
            Set the location (only position is used)
        */
        void setGlobalPose(const Eigen::Matrix4f& pose);

        /*!
            Returns Id of the surface vertex which is nearest to line through center of sphere and pose
            \param pose in global coordinate system, (center of sphere is at globalPose)
        */
        int getNearestVertexId(const Eigen::Matrix4f& pose);
        int getNearestVertexIdVec(const Eigen::Vector3f& pos);

        /*!
            Returns Id of the surface face which is nearest to line through center of sphere and pose
            \param pose in global coordinate system, (center of sphere is at globalPose)
        */
        int getNearestFaceId(const Eigen::Matrix4f& pose);
        int getNearestFaceIdVec(const Eigen::Vector3f& pos);

        void removeCSpaceNode(const Eigen::Vector3f& cartPose, CSpaceNodePtr node);
        void removeCSpaceNode(int faceId, CSpaceNodePtr node);

        /*!
        Add a node of the RRT with corresponding Cartesian pose. The pose should be the resulting EEF's pose.
        */
        void addCSpaceNode(const Eigen::Vector3f& cartPos, CSpaceNodePtr node);
        void addCSpaceNode(int faceId, CSpaceNodePtr node);

        //! Reset
        void clearCSpaceNodeMapping();

        /*!
            Return a RRT node, so that the corresponding Cartesian poses are uniformly sampled over the sphere.
            Removes the selected RRT node from the internal data.
            This method just checks loops faces of the sphere, so that the result is approximated.
        */
        CSpaceNodePtr getGoodRatedNode(int loops = 30);


    private:

        struct CSpaceNodeMapping
        {
            int count;
            std::vector<CSpaceNodePtr> cspaceNodes;
        };

        Eigen::Matrix4f globalPose;
        VirtualRobot::SphereApproximator::SphereApproximation sphere;
        VirtualRobot::SphereApproximatorPtr sphereGenerator;

        std::map<int, CSpaceNodeMapping> faceIdToCSpaceNodesMapping;
        std::vector<int> activeFaces;
    };

}

#endif /* _SABA_APPROACH_DISCRETIZATION_H_ */
