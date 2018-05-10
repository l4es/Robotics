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
#ifndef _VirtualRobot_MathTools_h_
#define _VirtualRobot_MathTools_h_

#include "VirtualRobotImportExport.h"

#include <algorithm>
#include <string>
#include <list>
#include <sstream>
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <float.h>

namespace VirtualRobot
{

    namespace MathTools
    {
        /************************************************************************/
        /* QUATERNIONS                                                          */
        /************************************************************************/

        struct Quaternion
        {
            Quaternion()
            {
                x = y = z = 0.0f;
                w = 1.0f;
            }
            float x, y, z, w;
        };


        /*!
            Return rotation that converts vector from to vector to.
        */
        Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT getRotation(const Eigen::Vector3f& from, const Eigen::Vector3f& to);

        //! get the corresponding angle of rotation that is defined by the quaternion (radian)
        float VIRTUAL_ROBOT_IMPORT_EXPORT getAngle(const Quaternion& q);

        //! Return the quaternion that defines the difference between the two given rotations
        Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT getDelta(const Quaternion& q1, const Quaternion& q2);

        //! Return the inverse quaternion
        Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT getInverse(const Quaternion& q);

        //! Return the conjugated quaternion
        Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT getConjugated(const Quaternion& q);

        //! Returns q1*q2
        Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT multiplyQuaternions(const Quaternion& q1, const Quaternion& q2);

        //! returns q1 dot q2
        float VIRTUAL_ROBOT_IMPORT_EXPORT getDot(const Quaternion& q1, const Quaternion& q2);

        //! Computes mean orientation of quaternions
        MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT getMean(std::vector<MathTools::Quaternion> quaternions);

        /*!
        Apply the slerp interpolation
        \param q1 The first quaternion
        \param q2 The second quaternion
        \param alpha A value between 0 and 1
        \return The intermediate quaternion
        */
        MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT slerp(const MathTools::Quaternion& q1, const MathTools::Quaternion& q2, float alpha);

        /************************************************************************/
        /* SPHERICAL COORDINATES                                                */
        /************************************************************************/
        struct SphericalCoord
        {
            float r, phi, theta;
        };

        SphericalCoord VIRTUAL_ROBOT_IMPORT_EXPORT toSphericalCoords(const Eigen::Vector3f& pos);
        Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT toPosition(const SphericalCoord& sc);


        /************************************************************************/
        /* CONVERTIONS                                                          */
        /************************************************************************/

        /*!
            Convert rpy values to a 3x3 rotation matrix and store it to the rotational component of the given homogeneous matrix.
            The translation is set to zero.
        */
        void VIRTUAL_ROBOT_IMPORT_EXPORT rpy2eigen4f(float r, float p, float y, Eigen::Matrix4f& m);
        void VIRTUAL_ROBOT_IMPORT_EXPORT posrpy2eigen4f(const float x[6], Eigen::Matrix4f& m);
        void VIRTUAL_ROBOT_IMPORT_EXPORT posrpy2eigen4f(const Eigen::Vector3f& pos, const Eigen::Vector3f& rpy, Eigen::Matrix4f& m);

        /*!
            Convert homogeneous matrix to translation and rpy rotation.
            \param m The matrix to be converted
            \param x The result is stored in this float array (x,y,z,roll,pitch,yaw)
        */
        void VIRTUAL_ROBOT_IMPORT_EXPORT eigen4f2rpy(const Eigen::Matrix4f& m, float x[6]);
        void VIRTUAL_ROBOT_IMPORT_EXPORT eigen4f2rpy(const Eigen::Matrix4f& m, Eigen::Vector3f& storeRPY);

        /*!
            Convert quaternion values to a 3x3 rotation matrix and store it to the rotational component of the result.
            The translational part of m is zero
            \return Homogeneous matrix representing the rotation of q.
        */
        Eigen::Matrix4f VIRTUAL_ROBOT_IMPORT_EXPORT quat2eigen4f(float x, float y, float z, float w);
        Eigen::Matrix4f VIRTUAL_ROBOT_IMPORT_EXPORT quat2eigen4f(const Quaternion q);

        Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT eigen4f2quat(const Eigen::Matrix4f& m);

        Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT getTranslation(const Eigen::Matrix4f& m);

        void VIRTUAL_ROBOT_IMPORT_EXPORT eigen4f2axisangle(const Eigen::Matrix4f& m, Eigen::Vector3f& storeAxis, float& storeAngle);
        Eigen::Matrix4f VIRTUAL_ROBOT_IMPORT_EXPORT axisangle2eigen4f(const Eigen::Vector3f& axis, float angle);
        Eigen::Matrix3f VIRTUAL_ROBOT_IMPORT_EXPORT axisangle2eigen3f(const Eigen::Vector3f& axis, float angle);
        Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT axisangle2quat(const Eigen::Vector3f& axis, float angle);

        /*!
            Compute the delta of two poses.
            \param m1 The first pose.
            \param m2 The second pose.
            \param storeDetalPos The position delta is stored here.
            \param storeDeltaRot The orientation delta is stored here [radian]
        */
        void VIRTUAL_ROBOT_IMPORT_EXPORT getDelta(const Eigen::Matrix4f& m1, const Eigen::Matrix4f& m2, float& storeDetalPos, float& storeDeltaRot);


        float VIRTUAL_ROBOT_IMPORT_EXPORT rad2deg(float rad);
        float VIRTUAL_ROBOT_IMPORT_EXPORT deg2rad(float deg);

        /************************************************************************/
        /* GEOMETRY                                                             */
        /************************************************************************/
        struct Plane
        {
            Plane()
            {
                p = Eigen::Vector3f::Zero();
                n = Eigen::Vector3f::UnitZ();
            }

            Plane(const Eigen::Vector3f& point, const Eigen::Vector3f& normal)
            {
                p = point;
                n = normal;
                n.normalize();
            }

            Plane(const Plane& plane)
            {
                this->p = plane.p;
                this->n = plane.n;
            }

            Eigen::Vector3f p;  // point
            Eigen::Vector3f n;  // normal (unit length)
        };

        //! Create a floor plane
        Plane VIRTUAL_ROBOT_IMPORT_EXPORT getFloorPlane();

        template<typename VectorT>
        struct BaseLine
        {
            BaseLine() {}

            BaseLine(const VectorT& point, const VectorT& dir)
            {
                p = point;
                d = dir;

                if (d.norm() > 1e-9)
                {
                    d.normalize();
                }
            }

            BaseLine(const BaseLine<VectorT>& line)
                : p(line.p)
                , d(line.d)
            {
            }

            bool isValid() const
            {
                return d.norm() > 1e-9;
            }

            VectorT p;  // point
            VectorT d;  // direction (unit length)
        };

        typedef BaseLine<Eigen::Vector3f> Line;
        typedef BaseLine<Eigen::Vector2f> Line2D;

        struct Segment
        {
            Segment()
            {
                p0.setZero();
                p1.setZero();
            }
            Segment(const Eigen::Vector3f& point0, const Eigen::Vector3f& point1)
            {
                p0 = point0;
                p1 = point1;
            }
            Eigen::Vector3f p0;
            Eigen::Vector3f p1;
        };

        struct OOBB
        {
            OOBB()
            {
                minBB.setZero();
                maxBB.setZero();
                pose.setIdentity();
            }
            OOBB(const Eigen::Vector3f& minLocal, const Eigen::Vector3f& maxLocal, const Eigen::Matrix4f& globalPose)
            {
                minBB = minLocal;
                maxBB = maxLocal;
                pose = globalPose;
            }
            //! Returns the 8 bounding box points transformed to global frame
            std::vector<Eigen::Vector3f> getOOBBPoints() const
            {
                Eigen::Vector3f result[8];
                std::vector<Eigen::Vector3f> result3;
                result[0] << minBB(0), minBB(1), minBB(2);
                result[1] << maxBB(0), minBB(1), minBB(2);
                result[2] << minBB(0), maxBB(1), minBB(2);
                result[3] << maxBB(0), maxBB(1), minBB(2);
                result[4] << minBB(0), minBB(1), maxBB(2);
                result[5] << maxBB(0), minBB(1), maxBB(2);
                result[6] << minBB(0), maxBB(1), maxBB(2);
                result[7] << maxBB(0), maxBB(1), maxBB(2);
                Eigen::Matrix4f m;
                m.setIdentity();

                for (int i = 0; i < 8; i++)
                {
                    m.block(0, 3, 3, 1) = result[i];
                    m = pose * m;
                    result3.push_back(m.block(0, 3, 3, 1));
                }

                return result3;
            }
            //! Returns the 12 segments of the bounding box (in global frame)
            std::vector<Segment> getSegments() const
            {
                std::vector<Eigen::Vector3f> oobbPoint = getOOBBPoints();
                std::vector<Segment> result;

                result.push_back(Segment(oobbPoint[0], oobbPoint[1]));
                result.push_back(Segment(oobbPoint[0], oobbPoint[2]));
                result.push_back(Segment(oobbPoint[2], oobbPoint[3]));
                result.push_back(Segment(oobbPoint[1], oobbPoint[3]));

                result.push_back(Segment(oobbPoint[4], oobbPoint[5]));
                result.push_back(Segment(oobbPoint[4], oobbPoint[6]));
                result.push_back(Segment(oobbPoint[5], oobbPoint[7]));
                result.push_back(Segment(oobbPoint[6], oobbPoint[7]));

                result.push_back(Segment(oobbPoint[2], oobbPoint[6]));
                result.push_back(Segment(oobbPoint[0], oobbPoint[4]));
                result.push_back(Segment(oobbPoint[1], oobbPoint[5]));
                result.push_back(Segment(oobbPoint[3], oobbPoint[7]));

                return result;
            };

            void changeCoordSystem(const Eigen::Matrix4f& newGlobalPose)
            {
                Eigen::Vector3f result[8];
                std::vector<Eigen::Vector3f> result3;
                result[0] << minBB(0), minBB(1), minBB(2);
                result[1] << maxBB(0), minBB(1), minBB(2);
                result[2] << minBB(0), maxBB(1), minBB(2);
                result[3] << maxBB(0), maxBB(1), minBB(2);
                result[4] << minBB(0), minBB(1), maxBB(2);
                result[5] << maxBB(0), minBB(1), maxBB(2);
                result[6] << minBB(0), maxBB(1), maxBB(2);
                result[7] << maxBB(0), maxBB(1), maxBB(2);
                Eigen::Matrix4f m;

                for (int i = 0; i < 8; i++)
                {
                    m.setIdentity();
                    m.block(0, 3, 3, 1) = result[i];
                    // to global
                    m = pose * m;
                    // to new coord system (local in new coord system)
                    m = newGlobalPose.inverse() * m;

                    result3.push_back(m.block(0, 3, 3, 1));
                }

                // now find min max values
                minBB << FLT_MAX, FLT_MAX, FLT_MAX;
                maxBB << -FLT_MAX, -FLT_MAX, -FLT_MAX;

                for (int i = 0; i < 8; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        if (result3[i](j) < minBB(j))
                        {
                            minBB(j) = result3[i](j);
                        }

                        if (result3[i](j) > maxBB(j))
                        {
                            maxBB(j) = result3[i](j);
                        }
                    }
                }

                pose = newGlobalPose;
            }

            // the bounding box is defined via min and max values (in local frame)
            Eigen::Vector3f minBB;
            Eigen::Vector3f maxBB;

            Eigen::Matrix4f pose; // the transformation of the bounding box in global frame.
        };

        /*!
            Convenient structs for handling 3D/6D vertices, faces and convex hulls.
        */
        struct ContactPoint
        {
            Eigen::Vector3f p;  // point
            Eigen::Vector3f n;  // normal
            float force;
        };

        struct TriangleFace
        {
            TriangleFace()
                : id1(UINT_MAX), id2(UINT_MAX), id3(UINT_MAX),
                  idColor1(UINT_MAX), idColor2(UINT_MAX), idColor3(UINT_MAX),
                  idNormal1(UINT_MAX), idNormal2(UINT_MAX), idNormal3(UINT_MAX),
                  idMaterial(UINT_MAX) {}

            /**
             * Flips the orientation of the contained vertex and the normal.
             */
            void flipOrientation()
            {
                std::swap(id3, id1);
                normal *= -1.0f;
            }
            void set(unsigned int id1, unsigned int id2, unsigned int id3)
            {
                this->id1 = id1;
                this->id2 = id2;
                this->id3 = id3;
            }
            void setColor(unsigned int idColor1, unsigned int idColor2, unsigned int idColor3)
            {
                this->idColor1 = idColor1;
                this->idColor2 = idColor2;
                this->idColor3 = idColor3;
            }
            void setNormal(unsigned int idNormal1, unsigned int idNormal2, unsigned int idNormal3)
            {
                this->idNormal1 = idNormal1;
                this->idNormal2 = idNormal2;
                this->idNormal3 = idNormal3;
            }
            void setMaterial(unsigned int idMaterial)
            {
                this->idMaterial = idMaterial;
            }

            // id == position in vertex array
            unsigned int id1;
            unsigned int id2;
            unsigned int id3;

            // idColor == position in color array
            unsigned int idColor1;
            unsigned int idColor2;
            unsigned int idColor3;

            //idNormal == position in normal array
            unsigned int idNormal1;
            unsigned int idNormal2;
            unsigned int idNormal3;

            // idMaterial == position in material array
            unsigned int idMaterial;

            // per face normal (used when no idNormals are given)
            Eigen::Vector3f normal;
        };
        struct TriangleFace6D
        {
            int id[6];// position in vertice vector (x,y,z,nx,ny,nz)
            ContactPoint normal;

            // these values are set by the ConvexHull algorithm (see GraspStudio)
            float distNormZero;     // distance of facet to origin
            float distNormCenter;   // distance of facet to center of convex hull
            float distPlaneZero;    // distance of plane defined by facet to origin
            float distPlaneCenter;  // distance of plane defined by facet to center of convex hull
            float offset;           // offset value of facet, determined by qhull
        };

        enum IntersectionResult
        {
            eParallel,
            eNoIntersection,
            eIntersection
        };


        //! Get the projected point in 3D
        Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT projectPointToPlane(const Eigen::Vector3f& point, const Plane& plane);

        //! Get the intersection line of two planes. If planes are parallel, the resulting line is invalid, i.e. has a zero direction vector!
        Line VIRTUAL_ROBOT_IMPORT_EXPORT intersectPlanes(const Plane& p1, const Plane& p2);

        /*!
            Get the intersection of segment and plane.
            \param segment The segment
            \param plane The plane
            \param storeResult In case the intersection exists, the result is stored here
            \result If there is no intersection the result is eNoIntersection. In case the result is eIntersection, the resulting intersection point is stored in storeResult.
        */
        IntersectionResult VIRTUAL_ROBOT_IMPORT_EXPORT intersectSegmentPlane(const Segment& segment, const Plane& plane, Eigen::Vector3f& storeResult);

        /*!
        Intersect an object oriented bounding box (oobb) with a plane.
        \param oobb The oobb
        \param plane The plane
        \param storeResult In case an intersection exists, the intersection area is defined by these points
        \result If the oobb does not intersect eNoIntersection is returned, otherwise eIntersection.
        */
        IntersectionResult VIRTUAL_ROBOT_IMPORT_EXPORT intersectOOBBPlane(const OOBB& oobb, const Plane& plane, std::vector<Eigen::Vector3f>& storeResult);

        //! Returns nearest point to p on line l
        template<typename VectorT>
        inline VectorT nearestPointOnLine(const BaseLine<VectorT>& l, const VectorT& p)
        {
            if (!l.isValid())
            {
                return VectorT::Zero();
            }

            VectorT lp = p - l.p;

            float lambda = l.d.dot(lp);

            VectorT res = l.p + lambda * l.d;
            return res;
        }

        //! Returns nearest point to p on segment given by the two end points
        template<typename VectorT>
        inline VectorT nearestPointOnSegment(const VectorT& firstEndPoint, const VectorT& secondEndPoint, const VectorT& p)
        {
            VectorT direction = secondEndPoint - firstEndPoint;
            direction /= direction.norm();
            const BaseLine<VectorT> l(firstEndPoint, direction);
            VR_ASSERT(l.isValid());

            VectorT onLine = nearestPointOnLine<VectorT>(l, p);
            double alpha = (onLine - firstEndPoint).dot(direction);

            // point not on segment, below first end point
            if (alpha < 0)
            {
                return firstEndPoint;
            }

            // point not on segment, above second end point
            if (alpha > (secondEndPoint - firstEndPoint).norm())
            {
                return secondEndPoint;
            }

            return onLine;
        }

        //! Returns the distance of vector p to line l
        template<typename VectorT>
        inline float distPointLine(const BaseLine<VectorT>& l, const VectorT& p)
        {
            if (!l.isValid())
            {
                return -1.0f;
            }

            VectorT p2 = nearestPointOnLine<VectorT>(l, p);
            return (p2 - p).norm();
        }

        //! Returns the distance of vector p to segment given by the two end points
        template<typename VectorT>
        inline float distPointSegment(const VectorT& firstEndPoint, const VectorT& secondEndPoint, const VectorT& p)
        {
            return (p - nearestPointOnSegment(firstEndPoint, secondEndPoint, p)).norm();
        }

        //! Check if three points are collinear
        bool VIRTUAL_ROBOT_IMPORT_EXPORT collinear(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3);

        /*!
            assuming all points to be in a plane. Returns normal of this plane.
        */
        Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT findNormal(const std::vector<Eigen::Vector3f>& points);

        //! Get the projected point in 2D (local coordinate system of the plane)
        Eigen::Vector2f VIRTUAL_ROBOT_IMPORT_EXPORT projectPointToPlane2D(const Eigen::Vector3f& point, const Plane& plane);

        //! Get the corresponding point in 3D
        Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT planePoint3D(const Eigen::Vector2f& pointLocal, const Plane& plane);

        float VIRTUAL_ROBOT_IMPORT_EXPORT getDistancePointPlane(const Eigen::Vector3f& point, const Plane& plane);

        /*!
            This method can be used to multiply a 3d position with a matrix
            result = m * pos
        */
        Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT transformPosition(const Eigen::Vector3f& pos, const Eigen::Matrix4f& m);

        /*!
            This method can be used to multiply a 2d position with a matrix
            result = m * pos
        */
        Eigen::Vector2f VIRTUAL_ROBOT_IMPORT_EXPORT transformPosition(const Eigen::Vector2f& pos, const Eigen::Matrix4f& m);

        //! Get a random point inside the triangle that is spanned by v1, v2 and v3
        Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT randomPointInTriangle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3);

        /*!
            Returns true, if point is on the side of plane in which the normal vector is pointing.
        */
        bool VIRTUAL_ROBOT_IMPORT_EXPORT onNormalPointingSide(const Eigen::Vector3f& point, const Plane& p);

        /*!
            Returns angle between v1 and v2 [rad].
        */
        float VIRTUAL_ROBOT_IMPORT_EXPORT getAngle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);

        /*!
            This method unites the translational and rotational difference of two Cartesian poses to one metric value.
            The translational distance is the norm in mm (counts 1)
            The rotational distance is the approximated angle between the two orientations in degrees (counts 3, or whatever you specify with rotInfluence)
            In the standard setting -> 3mm equals 1 degree in this metric
            The angle is approximated by replacing the costly eq alpha=2*acos(q0) with a linear term: alpha = 180-(q0+1)*90
        */
        float VIRTUAL_ROBOT_IMPORT_EXPORT getCartesianPoseDiff(const Eigen::Matrix4f& p1, const Eigen::Matrix4f& p2, float rotInfluence = 3.0f);


        /************************************************************************/
        /* CONVEX HULLS                                                         */
        /* More convex hull methods can be found in the GarspStudio lib         */
        /************************************************************************/

        struct Segment2D
        {
            int id1;
            int id2;
        };

        struct ConvexHull2D
        {
            std::vector<Eigen::Vector2f> vertices;
            std::vector<Segment2D> segments;
        };
        typedef boost::shared_ptr<ConvexHull2D> ConvexHull2DPtr;

        struct ConvexHull3D
        {
            std::vector<Eigen::Vector3f> vertices;
            std::vector<TriangleFace> faces;
            float volume;

            Eigen::Vector3f center;
            float maxDistFacetCenter; // maximum distance of faces to center
        };
        typedef boost::shared_ptr<ConvexHull3D> ConvexHull3DPtr;

        struct ConvexHull6D
        {
            std::vector<ContactPoint>   vertices;
            std::vector<TriangleFace6D> faces;
            float volume;
            ContactPoint center;
        };
        typedef boost::shared_ptr<ConvexHull6D> ConvexHull6DPtr;


        // Copyright 2001, softSurfer (www.softsurfer.com)
        // This code may be freely used and modified for any purpose
        // providing that this copyright notice is included with it.
        // SoftSurfer makes no warranty for this code, and cannot be held
        // liable for any real or imagined damage resulting from its use.
        // Users of this code must verify correctness for their application.
        // isLeft(): tests if a point is Left|On|Right of an infinite line.
        //    Input:  three points P0, P1, and P2
        //    Return: >0 for P2 left of the line through P0 and P1
        //            =0 for P2 on the line
        //            <0 for P2 right of the line
        //    See: the January 2001 Algorithm on Area of Triangles
        inline float isLeft(Eigen::Vector2f P0, Eigen::Vector2f P1, Eigen::Vector2f P2)
        {
            return (P1(0) - P0(0)) * (P2(1) - P0(1)) - (P2(0) - P0(0)) * (P1(1) - P0(1));
        }

        ConvexHull2DPtr VIRTUAL_ROBOT_IMPORT_EXPORT createConvexHull2D(const std::vector< Eigen::Vector2f > &points);
        Eigen::Vector2f VIRTUAL_ROBOT_IMPORT_EXPORT getConvexHullCenter(ConvexHull2DPtr ch);
        bool VIRTUAL_ROBOT_IMPORT_EXPORT isInside(const Eigen::Vector2f& p, ConvexHull2DPtr hull);

        /*!
            Sort points according to their first coordinate. If two points share the first coord, the second one is used to decide which is "smaller".
        */
        std::vector< Eigen::Vector2f > sortPoints(const std::vector< Eigen::Vector2f >& points);



        /************************************************************************/
        /* BASIS HELPERS                                                        */
        /************************************************************************/
        bool VIRTUAL_ROBOT_IMPORT_EXPORT ensureOrthonormalBasis(Eigen::Vector3f& x, Eigen::Vector3f& y, Eigen::Vector3f& z);


        //! perform the Gram-Schmidt method for building an orthonormal basis
        bool VIRTUAL_ROBOT_IMPORT_EXPORT GramSchmidt(std::vector< Eigen::VectorXf >& basis);

        //! Searches a vector that is linearly independent of the given basis.
        bool VIRTUAL_ROBOT_IMPORT_EXPORT randomLinearlyIndependentVector(const std::vector< Eigen::VectorXf > basis, Eigen::VectorXf& storeResult);

        //! Checks if m contains a col vector equal to v (distance up to 1e-8 are considered as equal)
        bool VIRTUAL_ROBOT_IMPORT_EXPORT containsVector(const Eigen::MatrixXf& m, const Eigen::VectorXf& v);


        /*!
            Computes the matrix describing the basis transformation from basis formed by vectors in basisSrc to basisDst.
            So v_{Src} can be transformed to v_{Dst}, given as linear combination of basisDst vectors, by
            v_{Dst} = T v_{Src} with T is the BasisTransformationMatrix given by this method.
            \param basisSrc The initial basis vectors.
            \param basisDst The final basis vectors.
            \return The transformation matrix T.
        */
        Eigen::MatrixXf VIRTUAL_ROBOT_IMPORT_EXPORT getBasisTransformation(const std::vector< Eigen::VectorXf >& basisSrc, const std::vector< Eigen::VectorXf >& basisDst);

        /*!
            Computes the matrix describing the basis transformation from basis formed by vectors in basisSrc to basisDst.
            So v_{Src} can be transformed to v_{Dst}, given as linear combination of basisDst vectors, by
            v_{Dst} = T v_{Src} with T is the BasisTransformationMatrix given by this method.
            \param basisSrc The column vectors are the initial basis vectors
            \param basisDst The column vectors are the final basis vectors.
            \return The transformation matrix T.
        */
        Eigen::MatrixXf VIRTUAL_ROBOT_IMPORT_EXPORT getBasisTransformation(const Eigen::MatrixXf& basisSrc, const Eigen::MatrixXf& basisDst);

        Eigen::VectorXf VIRTUAL_ROBOT_IMPORT_EXPORT getPermutation(const Eigen::VectorXf& inputA, const Eigen::VectorXf& inputB, unsigned int i);
        /************************************************************************/
        /* HELPERS and IO                                                       */
        /************************************************************************/

        int VIRTUAL_ROBOT_IMPORT_EXPORT pow_int(int a, int b);

        /*!
            Returns the Pseudo inverse matrix.
        */
        Eigen::MatrixXf VIRTUAL_ROBOT_IMPORT_EXPORT getPseudoInverse(const Eigen::MatrixXf& m, float tol = 1e-5f);
        /*!
            Returns the damped Pseudo inverse matrix.
        */
        Eigen::MatrixXf VIRTUAL_ROBOT_IMPORT_EXPORT getPseudoInverseDamped(const Eigen::MatrixXf& m, float lambda = 1.0f);

        /*!
            Check if all entries of v are valid numbers (i.e. all entries of v are not NaN and not INF)
        */
        bool VIRTUAL_ROBOT_IMPORT_EXPORT isValid(const Eigen::MatrixXf& v);
        void VIRTUAL_ROBOT_IMPORT_EXPORT print(const ContactPoint& p);
        void VIRTUAL_ROBOT_IMPORT_EXPORT print(const std::vector<ContactPoint>& points);
        void VIRTUAL_ROBOT_IMPORT_EXPORT print(const Eigen::VectorXf& v, bool endline = true);
        void VIRTUAL_ROBOT_IMPORT_EXPORT printMat(const Eigen::MatrixXf& m, bool endline = true);
        void VIRTUAL_ROBOT_IMPORT_EXPORT print(const std::vector<float>& v, bool endline = true);
        std::string VIRTUAL_ROBOT_IMPORT_EXPORT getTransformXMLString(const Eigen::Matrix4f& m, int tabs, bool skipMatrixTag = false);
        std::string VIRTUAL_ROBOT_IMPORT_EXPORT getTransformXMLString(const Eigen::Matrix4f& m, const std::string& tabs, bool skipMatrixTag = false);
        std::string VIRTUAL_ROBOT_IMPORT_EXPORT getTransformXMLString(const Eigen::Matrix3f& m, int tabs, bool skipMatrixTag = false);
        std::string VIRTUAL_ROBOT_IMPORT_EXPORT getTransformXMLString(const Eigen::Matrix3f& m, const std::string& tabs, bool skipMatrixTag = false);
        void VIRTUAL_ROBOT_IMPORT_EXPORT convertMM2M(const std::vector<ContactPoint> points, std::vector<ContactPoint>& storeResult);


    };
} // namespace VirtualRobot

#endif //_VirtualRobot_MathTools_h_
