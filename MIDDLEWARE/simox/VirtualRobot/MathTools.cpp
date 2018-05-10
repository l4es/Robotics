
#include "MathTools.h"
#include "VirtualRobotException.h"
#include <float.h>
#include <string.h>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <algorithm>



#include <Eigen/Geometry>

#define MIN_TO_ZERO 1e-6

namespace VirtualRobot
{

    /*
    void MathTools::Quat2RPY(float *PosQuat, float *storeResult)
    {
        if (storeResult==NULL || PosQuat==NULL)
            return;
        float tmpPose[7];
        tmpPose[0] = 0;
        tmpPose[1] = 0;
        tmpPose[2] = 0;
        tmpPose[3] = PosQuat[0];
        tmpPose[4] = PosQuat[1];
        tmpPose[5] = PosQuat[2];
        tmpPose[6] = PosQuat[3];
        SbMatrix m;
        PosQuat2SbMatrix(tmpPose,m);
        SbMatrix2PosRPY(m,tmpPose);
        storeResult[0] = tmpPose[3];
        storeResult[1] = tmpPose[4];
        storeResult[2] = tmpPose[5];
    }

    void MathTools::PosQuat2PosRPY(float *PosQuat, float *storePosRPY)
    {
        if (storePosRPY==NULL || PosQuat==NULL)
            return;
        SbMatrix m;
        PosQuat2SbMatrix(PosQuat,m);
        SbMatrix2PosRPY(m,storePosRPY);
    }

    bool MathTools::PosEulerZXZ2SbMatrix(const float* PosEulerZXZ, SbMatrix &storeResult)
    {
        float posRPY[6];
        if (!PosEulerZXZ2PosRPY(PosEulerZXZ,posRPY))
            return false;
        PosRPY2SbMatrix(posRPY,storeResult);
        return true;
    }

    bool MathTools::PosEulerZXZ2PosRPY(const float* PosEulerZXZ, float* PosRPY)
    {
        // Euler ZXZ to Matrix conversion
        Matrix res(3,3);
        const double psi = PosEulerZXZ[3];
        const double theta = PosEulerZXZ[4];
        const double phi = PosEulerZXZ[5];
        double sa, ca, sb, cb, sc, cc;

        sa = sin(psi);
        ca = cos(psi);
        sb = sin(theta);
        cb = cos(theta);
        sc = sin(phi);
        cc = cos(phi);

        res(1, 1) = ca * cc - sa * cb * sc;
        res(1, 2) = -ca * sc - sa * cb * cc;
        res(1, 3) = sa * sb;
        //res(1, 4) = 0.0;

        res(2, 1) = sa * cc + ca * cb * sc;
        res(2, 2) = -sa * sc + ca * cb * cc;
        res(2, 3) = -ca * sb;
        //res(2, 4) = 0.0;

        res(3, 1) = sb * sc;
        res(3, 2) = sb * cc;
        res(3, 3) = cb;
        //res(3, 4) = 0.0;



        { // Matrix To Roll-Pitch-Yaw conversion
            double alpha, beta, gamma;
            beta = atan2(-res(3, 1), sqrt(res(1, 1) * res(1, 1) + res(2, 1) * res(2, 1)));

            if (fabs(beta - M_PI_2) < 1e-4)
            {
                alpha = 0;
                gamma = atan2(res(1, 2), res(2, 2));
            }
            else if (fabs(beta - (-M_PI_2)) < 1e-4)
            {
                alpha = 0;
                gamma = -atan2(res(1, 2), res(2, 2));
            }
            else
            {
                alpha = atan2(res(2, 1) / cos(beta), res(1, 1) / cos(beta));
                gamma = atan2(res(3, 2) / cos(beta), res(3, 3) / cos(beta));
            }

            PosRPY[0] = PosEulerZXZ[0];
            PosRPY[1] = PosEulerZXZ[1];
            PosRPY[2] = PosEulerZXZ[2];
            PosRPY[3] = (float)gamma;
            PosRPY[4] = (float)beta;
            PosRPY[5] = (float)alpha;

        }
        return true;
    }


    void MathTools::SbMatrix2PosEulerZXZ(const SbMatrix& mat, float *storeResult)
    {
        if (storeResult==NULL)
            return;
        SbMatrix A = mat.transpose();

        storeResult[0] = A[0][3];
        storeResult[1] = A[1][3];
        storeResult[2] = A[2][3];
        if (A[2][2] < 1.0)
        {
            if (A[2][2]>-1.0)
            {
                storeResult[3] = atan2(A[0][2], -A[1][2]);
                storeResult[4] = acos(A[2][2]);
                storeResult[5] = atan2(A[2][0], A[2][1]);
            } else
            {
                storeResult[3] = -atan2(-A[0][1], A[0][0]);
                storeResult[4] = (float)M_PI;
                storeResult[5] = 0;
            }
        } else
        {
            storeResult[3] = 0;
            storeResult[4] = atan2(-A[0][1], A[0][0]);
            storeResult[5] = 0;
        }
    }

    */

    /*
    void MathTools::Rpy2Quat(const float *rpy, float *storeQuat, bool switchWToFront)
    {
        if (rpy==NULL || storeQuat==NULL)
            return;
        float posArray[6];
        SbMatrix resRPY;
        posArray[0] = 0.0f;
        posArray[1] = 0.0f;
        posArray[2] = 0.0f;
        posArray[3] = rpy[0];
        posArray[4] = rpy[1];
        posArray[5] = rpy[2];
        MathTools::PosRPY2SbMatrix(posArray,resRPY);
        MathTools::SbMatrix2Quat(resRPY,storeQuat,switchWToFront);
    }

    void MathTools::PosRPY2PosQuat(float *PosRPY, float *storePosQuat)
    {
        if (PosRPY==NULL || storePosQuat==NULL)
            return;
        SbMatrix resRPY;
        MathTools::PosRPY2SbMatrix(PosRPY,resRPY);
        MathTools::SbMatrix2PosQuat(resRPY,storePosQuat);
    }


    void MathTools::crossProduct (float *v1, float *v2, float *storeRes)
    {
        float a = v1[1]*v2[2] - v1[2]*v2[1];
        float b = v1[2]*v2[0] - v1[0]*v2[2];
        float c = v1[0]*v2[1] - v1[1]*v2[0];

        storeRes[0] = a;
        storeRes[1] = b;
        storeRes[2] = c;
    }
    */


    void MathTools::eigen4f2rpy(const Eigen::Matrix4f& m, float x[6])
    {
        //Eigen::Matrix4f A = mat.transpose();
        float alpha, beta, gamma;
        beta = atan2(-m(2, 0),  sqrtf(m(0, 0) * m(0, 0) + m(1, 0) * m(1, 0)));

        if (fabs(beta - (float)M_PI * 0.5f) < 1e-10)
        {
            alpha = 0;
            gamma = atan2(m(0, 1), m(1, 1));
        }
        else if (fabs(beta + (float)M_PI * 0.5f) < 1e-10)
        {
            alpha = 0;
            gamma = -atan2(m(0, 1), m(1, 1));
        }
        else
        {
            float cb = 1.0f / cosf(beta);
            alpha = atan2(m(1, 0) * cb, m(0, 0) * cb);
            gamma = atan2(m(2, 1) * cb, m(2, 2) * cb);
        }

        x[0] = m(0, 3);
        x[1] = m(1, 3);
        x[2] = m(2, 3);
        x[3] = gamma;
        x[4] = beta;
        x[5] = alpha;
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::eigen4f2rpy(const Eigen::Matrix4f& m, Eigen::Vector3f& storeRPY)
    {
        float x[6];
        eigen4f2rpy(m, x);
        storeRPY(0) = x[3];
        storeRPY(1) = x[4];
        storeRPY(2) = x[5];
    }



    void MathTools::rpy2eigen4f(float r, float p, float y, Eigen::Matrix4f& m)
    {
        float salpha, calpha, sbeta, cbeta, sgamma, cgamma;

        sgamma = sinf(r);
        cgamma = cosf(r);
        sbeta = sinf(p);
        cbeta = cosf(p);
        salpha = sinf(y);
        calpha = cosf(y);

        m(0, 0) = (float)(calpha * cbeta);
        m(0, 1) = (float)(calpha * sbeta * sgamma - salpha * cgamma);
        m(0, 2) = (float)(calpha * sbeta * cgamma + salpha * sgamma);
        m(0, 3) = 0; //x

        m(1, 0) = (float)(salpha * cbeta);
        m(1, 1) = (float)(salpha * sbeta * sgamma + calpha * cgamma);
        m(1, 2) = (float)(salpha * sbeta * cgamma - calpha * sgamma);
        m(1, 3) = 0; //y

        m(2, 0) = (float) - sbeta;
        m(2, 1) = (float)(cbeta * sgamma);
        m(2, 2) = (float)(cbeta * cgamma);
        m(2, 3) = 0; //z

        m(3, 0) = 0;
        m(3, 1) = 0;
        m(3, 2) = 0;
        m(3, 3) = 1.0f;

    }


    void MathTools::posrpy2eigen4f(const float x[6], Eigen::Matrix4f& m)
    {
        rpy2eigen4f(x[3], x[4], x[5], m);
        m(0, 3) = x[0];
        m(1, 3) = x[1];
        m(2, 3) = x[2];
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::posrpy2eigen4f(const Eigen::Vector3f& pos, const Eigen::Vector3f& rpy, Eigen::Matrix4f& m)
    {
        float x[6];

        for (int i = 0; i < 3; i++)
        {
            x[i] = pos(i);
            x[i + 3] = rpy(i);
        }

        posrpy2eigen4f(x, m);
    }


    Eigen::Vector3f MathTools::getTranslation(const Eigen::Matrix4f& m)
    {
        return m.block(0, 3, 3, 1);
    }

    float MathTools::getDistancePointPlane(const Eigen::Vector3f& point, const Plane& plane)
    {
        return plane.n.dot(point - plane.p);
    }

    Eigen::Vector3f MathTools::projectPointToPlane(const Eigen::Vector3f& point, const Plane& plane)
    {
        float distance = getDistancePointPlane(point, plane);
        return point - distance * plane.n;
    }


    // Copyright 2001, softSurfer (www.softsurfer.com)
    // This code may be freely used and modified for any purpose
    // providing that this copyright notice is included with it.
    // SoftSurfer makes no warranty for this code, and cannot be held
    // liable for any real or imagined damage resulting from its use.
    // Users of this code must verify correctness for their application.
    MathTools::ConvexHull2DPtr MathTools::createConvexHull2D(const std::vector<Eigen::Vector2f> &points)
    {
        std::vector< Eigen::Vector2f > P = sortPoints(points);
        int n = (int)P.size();
        THROW_VR_EXCEPTION_IF(n < 3, "Could not generate convex hull with " << n << " points...");
        std::vector< Eigen::Vector2f > H;
        H.resize(n * 2);

        // the output array H[] will be used as the stack
        int    bot = 0, top = (-1); // indices for bottom and top of the stack
        int    i;                // array scan index

        // Get the indices of points with min x-coord and min|max y-coord
        int minmin = 0, minmax;
        float xmin = P[0](0);

        for (i = 1; i < n; i++)
            if (P[i](0) != xmin)
            {
                break;
            }

        minmax = i - 1;

        if (minmax == n - 1)
        {
            // degenerate case: all x-coords == xmin
            H[++top] = P[minmin];

            if (P[minmax](1) != P[minmin](1)) // a nontrivial segment
            {
                H[++top] = P[minmax];
            }

            H[++top] = P[minmin];           // add polygon endpoint

            // create 2d convex hull
            ConvexHull2DPtr hull(new ConvexHull2D());

            for (int u = 0; u < top + 1; u++)
            {
                hull->vertices.push_back(H[u]);
                Segment2D s;

                if (u != top)
                {
                    s.id1 = u;
                    s.id2 = u + 1;
                }
                else
                {
                    s.id1 = u;
                    s.id2 = 0;
                }

                hull->segments.push_back(s);
            }

            return hull;
        }

        // Get the indices of points with max x-coord and min|max y-coord
        int maxmin, maxmax = n - 1;
        float xmax = P[n - 1](0);

        for (i = n - 2; i >= 0; i--)
            if (P[i](0) != xmax)
            {
                break;
            }

        maxmin = i + 1;

        // Compute the lower hull on the stack H
        H[++top] = P[minmin];      // push minmin point onto stack
        i = minmax;

        while (++i <= maxmin)
        {
            // the lower line joins P[minmin] with P[maxmin]
            if (isLeft(P[minmin], P[maxmin], P[i]) >= 0 && i < maxmin)
            {
                continue;    // ignore P[i] above or on the lower line
            }

            while (top > 0)        // there are at least 2 points on the stack
            {
                // test if P[i] is left of the line at the stack top
                if (isLeft(H[top - 1], H[top], P[i]) > 0)
                {
                    break;    // P[i] is a new hull vertex
                }
                else
                {
                    top--;    // pop top point off stack
                }
            }

            H[++top] = P[i];       // push P[i] onto stack
        }

        // Next, compute the upper hull on the stack H above the bottom hull
        if (maxmax != maxmin)      // if distinct xmax points
        {
            H[++top] = P[maxmax];    // push maxmax point onto stack
        }

        bot = top;                 // the bottom point of the upper hull stack
        i = maxmin;

        while (--i >= minmax)
        {
            // the upper line joins P[maxmax] with P[minmax]
            if (isLeft(P[maxmax], P[minmax], P[i]) >= 0 && i > minmax)
            {
                continue;    // ignore P[i] below or on the upper line
            }

            while (top > bot)    // at least 2 points on the upper stack
            {
                // test if P[i] is left of the line at the stack top
                if (isLeft(H[top - 1], H[top], P[i]) > 0)
                {
                    break;    // P[i] is a new hull vertex
                }
                else
                {
                    top--;    // pop top point off stack
                }
            }

            H[++top] = P[i];       // push P[i] onto stack
        }

        if (minmax != minmin)
        {
            H[++top] = P[minmin];    // push joining endpoint onto stack
        }


        // create 2d convex hull
        ConvexHull2DPtr hull(new ConvexHull2D());

        for (int u = 0; u < top; u++)
        {
            hull->vertices.push_back(H[u]);
            Segment2D s;

            if (u != top - 1)
            {
                s.id1 = u;
                s.id2 = u + 1;
            }
            else
            {
                s.id1 = u;
                s.id2 = 0;
            }

            hull->segments.push_back(s);
        }

        return hull;
    }

    std::vector< Eigen::Vector2f > MathTools::sortPoints(const std::vector< Eigen::Vector2f >& points)
    {
        std::vector< Eigen::Vector2f > tmp = points;

        // Remove double entries
        for(auto point = tmp.begin(); point != tmp.end(); point++)
        {
            tmp.erase(std::remove_if(point+1, tmp.end(), [&](const Eigen::Vector2f &p) { return ((*point) - p).squaredNorm() < 1e-8; }), tmp.end());
        }

        std::sort(tmp.begin(), tmp.end(), [](const Eigen::Vector2f &a, const Eigen::Vector2f &b) { return a(0) < b(0) || (a(0) == b(0) && a(1) < b(1)); });
        return tmp;
    }

    bool MathTools::isInside(const Eigen::Vector2f& p, ConvexHull2DPtr hull)
    {
        if (!hull)
        {
            return false;
        }

        float qx, qy, rx, ry;
        float px = p(0);
        float py = p(1);

        for (size_t i = 0; i < hull->segments.size(); i++)
        {
            qx = hull->vertices[hull->segments[i].id1](0);
            qy = hull->vertices[hull->segments[i].id1](1);
            rx = hull->vertices[hull->segments[i].id2](0);
            ry = hull->vertices[hull->segments[i].id2](1);
            float x = (rx - qx) * (py - qy) - (px - qx) * (ry - qy);

            if (x <= 0)
            {
                if (x == 0) // colinear
                {
                    if ((px - qx) * (px - rx) > 0)
                    {
                        return false;
                    }

                    if ((py - qy) * (py - ry) > 0)
                    {
                        return false;
                    }

                    return true;
                }

                return false;
            }
        }

        return true;
    }

    MathTools::Quaternion MathTools::getRotation(const Eigen::Vector3f& from, const Eigen::Vector3f& to)
    {
        Eigen::Vector3f fromN = from;
        fromN.normalize();
        Eigen::Vector3f toN = to;
        toN.normalize();

        // rotation of the plane
        float d = fromN.dot(toN);
        Eigen::Vector3f crossvec = fromN.cross(toN);
        float crosslen = crossvec.norm();
        MathTools::Quaternion quat;

        if (crosslen == 0.0f)
        {
            // Parallel vectors
            if (d > 0.0f)
            {
                // same direction->nothing to do
            }
            else
            {
                // parallel and pointing in opposite direction
                // crossing with x axis.
                Eigen::Vector3f t = fromN.cross(Eigen::Vector3f(1.0f, 0.0f, 0.0f));

                // no->cross with y axis.
                if (t.norm() == 0.0f)
                {
                    t = fromN.cross(Eigen::Vector3f(0.0f, 1.0f, 0.0f));
                }

                t.normalize();
                quat.x = t[0];
                quat.y = t[1];
                quat.z = t[2];
                quat.w = 0.0f;
            }
        }
        else
        {
            // not parallel
            crossvec.normalize();
            // The fabs() wrapping is to avoid problems when d overflows
            crossvec *= (float)sqrt(0.5f * fabs(1.0f - d));
            quat.x = crossvec[0];
            quat.y = crossvec[1];
            quat.z = crossvec[2];
            quat.w = (float)sqrt(0.5 * fabs(1.0 + d));
        }

        return quat;
    }

    Eigen::Vector3f MathTools::planePoint3D(const Eigen::Vector2f& pointLocal, const Plane& plane)
    {
        // get transformation to plane with normal (0,0,1)
        Eigen::Vector3f normalUp(0, 0, 1.0f);

        Quaternion quat = getRotation(normalUp, plane.n);

        Eigen::Matrix4f rotationMatrix = quat2eigen4f(quat);
        rotationMatrix(0, 3) = plane.p(0);
        rotationMatrix(1, 3) = plane.p(1);
        rotationMatrix(2, 3) = plane.p(2);

        // transform point
        Eigen::Vector4f pt4(pointLocal(0), pointLocal(1), 0.0f, 1.0f);
        pt4 = rotationMatrix * pt4;

        // now we are in the global coordinate system
        Eigen::Vector3f res(pt4(0), pt4(1), pt4(2));
        return res;
    }

    Eigen::Vector2f MathTools::projectPointToPlane2D(const Eigen::Vector3f& point, const Plane& plane)
    {
        Eigen::Vector3f ptPlane = MathTools::projectPointToPlane(point, plane);

        // transform point to plane through plane.p with upNormal -> x,y coordinates are local coordinates
        Eigen::Vector3f normalUp(0, 0, 1.0f);

        Quaternion quat = getRotation(plane.n, normalUp);

        Eigen::Matrix4f rotationMatrix = quat2eigen4f(quat);
        rotationMatrix(0, 3) = -plane.p(0);
        rotationMatrix(1, 3) = -plane.p(1);
        rotationMatrix(2, 3) = -plane.p(2);

        // transform point
        Eigen::Vector4f pt4(ptPlane(0), ptPlane(1), ptPlane(2), 1.0f);
        pt4 = rotationMatrix * pt4;

        // now the z coordinate is 0 and we are in the local 2d coordinate system
        Eigen::Vector2f res(pt4(0), pt4(1));
        return res;

    }

    Eigen::Matrix4f MathTools::quat2eigen4f(const Quaternion q)
    {
        return quat2eigen4f(q.x, q.y, q.z, q.w);
    }


    Eigen::Matrix4f MathTools::quat2eigen4f(float x, float y, float z, float w)
    {

        Eigen::Matrix4f m;
        m.setIdentity();
        Eigen::Quaternionf q(w, x, y, z);
        Eigen::Matrix3f m3;
        m3 = q.toRotationMatrix();
        m.block(0, 0, 3, 3) = m3;
        return m;
        /*
        m(0,0) = w*w + x*x - y*y - z*z;
        m(1,0) = 2*x*y + 2*w*z;
        m(2,0) = 2*x*z - 2*w*y;
        //m(3,0) = 0.0f;

        m(0,1) = 2*x*y-2*w*z;
        m(1,1) = w*w - x*x + y*y - z*z;
        m(2,1) = 2*y*z + 2*w*x;
        //m(3,1) = 0.0f;

        m(0,2) = 2*x*z + 2*w*y;
        m(1,2) = 2*y*z - 2*w*x;
        m(2,2) = w*w - x*x - y*y + z*z;
        //m(3,2) = 0.0f;

        //m(0,3) = 0.0f;
        //m(1,3) = 0.0f;
        //m(2,3) = 0.0f;
        //m(3,3) = w*w + x*x + y*y + z*z;*/
    }

    std::string MathTools::getTransformXMLString(const Eigen::Matrix3f& m, int tabs, bool skipMatrixTag)
    {
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        return getTransformXMLString(m, t, skipMatrixTag);
    }

    std::string MathTools::getTransformXMLString(const Eigen::Matrix4f& m, int tabs, bool skipMatrixTag)
    {
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        return getTransformXMLString(m, t, skipMatrixTag);
    }

    std::string MathTools::getTransformXMLString(const Eigen::Matrix4f& m, const std::string& tabs, bool skipMatrixTag)
    {
        std::stringstream ss;

        if (!skipMatrixTag)
        {
            ss << tabs << "<Matrix4x4>\n";
        }

        ss << tabs << "\t<row1 c1='" << m(0, 0) << "' c2='" << m(0, 1) << "' c3='" << m(0, 2) << "' c4='" << m(0, 3) << "'/>\n";
        ss << tabs << "\t<row2 c1='" << m(1, 0) << "' c2='" << m(1, 1) << "' c3='" << m(1, 2) << "' c4='" << m(1, 3) << "'/>\n";
        ss << tabs << "\t<row3 c1='" << m(2, 0) << "' c2='" << m(2, 1) << "' c3='" << m(2, 2) << "' c4='" << m(2, 3) << "'/>\n";
        ss << tabs << "\t<row4 c1='" << m(3, 0) << "' c2='" << m(3, 1) << "' c3='" << m(3, 2) << "' c4='" << m(3, 3) << "'/>\n";

        if (!skipMatrixTag)
        {
            ss << tabs << "</Matrix4x4>\n";
        }

        return ss.str();
    }

    std::string MathTools::getTransformXMLString(const Eigen::Matrix3f& m, const std::string& tabs, bool skipMatrixTag)
    {
        std::stringstream ss;

        if (!skipMatrixTag)
        {
            ss << tabs << "<Matrix3x3>\n";
        }

        ss << tabs << "\t<row1 c1='" << m(0, 0) << "' c2='" << m(0, 1) << "' c3='" << m(0, 2) << "'/>\n";
        ss << tabs << "\t<row2 c1='" << m(1, 0) << "' c2='" << m(1, 1) << "' c3='" << m(1, 2) << "'/>\n";
        ss << tabs << "\t<row3 c1='" << m(2, 0) << "' c2='" << m(2, 1) << "' c3='" << m(2, 2) << "'/>\n";

        if (!skipMatrixTag)
        {
            ss << tabs << "</Matrix3x3>\n";
        }

        return ss.str();
    }

    Eigen::Vector3f MathTools::transformPosition(const Eigen::Vector3f& pos, const Eigen::Matrix4f& m)
    {
        Eigen::Vector4f t(pos.x(), pos.y(), pos.z(), 1);
        t = m * t;
        return t.head(3);
    }

    Eigen::Vector2f MathTools::transformPosition(const Eigen::Vector2f& pos, const Eigen::Matrix4f& m)
    {
        Eigen::Vector4f t(pos.x(), pos.y(), 0, 1);
        t = m * t;
        return t.head(2);
    }

    bool VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::onNormalPointingSide(const Eigen::Vector3f& point, const Plane& p)
    {
        return (point - p.p).dot(p.n) >= 0;
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::convertMM2M(const std::vector<ContactPoint> points, std::vector<ContactPoint>& storeResult)
    {
        float s = 1.0f / 1000.0f;
        storeResult.clear();
        std::vector<ContactPoint>::const_iterator iter1 = points.begin();

        while (iter1 != points.end())
        {
            ContactPoint tmp = *iter1;
            tmp.p *= s;
            storeResult.push_back(tmp);
            iter1++;
        }
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::print(const std::vector<float>& v, bool endline)
    {
        std::streamsize pr = std::cout.precision(2);
        std::cout << std::fixed;

        for (size_t i = 0; i < v.size(); i++)
        {
            cout << v[i];

            if (i != v.size() - 1)
            {
                cout << ",";
            }
        }

        if (endline)
        {
            cout << endl;
        }

        std::cout << std::resetiosflags(std::ios::fixed);
        std::cout.precision(pr);
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::print(const Eigen::VectorXf& v, bool endline)
    {
        std::streamsize pr = std::cout.precision(2);
        std::cout << std::fixed;

        for (int i = 0; i < v.rows(); i++)
        {
            cout << v(i);

            if (i != v.rows() - 1)
            {
                cout << ",";
            }
        }

        if (endline)
        {
            cout << endl;
        }

        std::cout << std::resetiosflags(std::ios::fixed);
        std::cout.precision(pr);
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::printMat(const Eigen::MatrixXf& m, bool endline)
    {
        std::streamsize pr = std::cout.precision(2);
        std::cout << std::fixed;

        for (int i = 0; i < m.rows(); i++)
        {
            for (int j = 0; j < m.cols(); j++)
            {
                cout << m(i, j);

                if (j != m.cols() - 1)
                {
                    cout << ",";
                }
            }

            cout << endl;
        }

        if (endline)
        {
            cout << endl;
        }

        std::cout << std::resetiosflags(std::ios::fixed);
        std::cout.precision(pr);
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::print(const ContactPoint& p)
    {
        std::streamsize pr = std::cout.precision(2);
        std::cout << std::fixed << "p:" << p.p(0) << "," << p.p(1) << "," << p.p(2) << ", n:" << p.n(0) << "," << p.n(1) << "," << p.n(2) << std::endl;
        std::cout << std::resetiosflags(std::ios::fixed);
        std::cout.precision(pr);
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::print(const std::vector<ContactPoint>& points)
    {
        for (size_t i = 0; i < points.size(); i++)
        {
            print(points[i]);
        }
    }

    Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::randomPointInTriangle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3)
    {
        float b0 = (float)rand() / (float)(RAND_MAX);
        float b1 = (1.0f - b0) * (float)rand() / (float)(RAND_MAX);
        float b2 = 1 - b0 - b1;

        return v1 * b0  + v2 * b1 + v3 * b2;
    }

    bool VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::isValid(const Eigen::MatrixXf& v)
    {
        return !(boost::math::isinf(v.maxCoeff()) || boost::math::isinf(-v.minCoeff()) || boost::math::isnan(v.sum()));
    }


    Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::findNormal(const std::vector<Eigen::Vector3f>& points)
    {
        Eigen::Vector3f result(0, 0, 1.0f);

        if (points.size() < 3)
        {
            return result;
        }

        for (unsigned int i = 2; i < points.size(); i++)
        {
            if (!collinear(points[i - 2], points[i - 1], points[i]))
            {
                result = (points[i - 1] - points[i - 2]).cross((points[i - 1] - points[i]));
                result.normalize();
                return result;
            }
        }

        // all points collinear
        return result;
    }

    bool VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::collinear(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2, const Eigen::Vector3f& p3)
    {
        float d = ((p2 - p1).cross((p2 - p3))).norm();
        return (d < 1e-6);
    }

    bool VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::containsVector(const Eigen::MatrixXf& m, const Eigen::VectorXf& v)
    {
        VR_ASSERT(m.rows() == v.rows());

        for (int i = 0; i < (int)m.cols(); i++)
        {
            if ((m.block(0, i, m.rows(), 1) - v).norm() < 1e-8)
            {
                return true;
            }
        }

        return false;
    }

    // creates an orthonormal basis out of the given basis
    // overwrites given basis
    bool VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::GramSchmidt(std::vector< Eigen::VectorXf >& basis)
    {
        int dim = basis.size();
        THROW_VR_EXCEPTION_IF(dim == 0, "Need basis vectors");

        for (int i = 0; i < dim; i++)
        {
            VR_ASSERT_MESSAGE((int)basis[i].rows() == dim, "Length of basis vectors is wrong");
        }

        int rows = basis[0].rows();
        Eigen::MatrixXf mBasis(rows, dim);

        for (int i = 0; i < (int)basis.size(); i++)
        {
            mBasis.block(0, i, rows, 1) = basis[i];
        }

        Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(mBasis);
        Eigen::MatrixXf basisVectors = lu_decomp.image(mBasis);
        int startWithDet = (int)(basisVectors.cols());

        if (startWithDet != dim)
        {
            std::vector< Eigen::VectorXf > tmpBasisV;

            // remain order of basis vectors
            for (int i = 0; i < dim; i++)
            {
                if (containsVector(basisVectors, basis[i]))
                {
                    tmpBasisV.push_back(basis[i]);
                }
            }

            for (int j = startWithDet; j < dim; j++)
            {
                Eigen::VectorXf newBasisVector;

                if (!randomLinearlyIndependentVector(tmpBasisV, newBasisVector))
                {
                    VR_ERROR << "Could not find linearly independent basis vector?! " << endl;

                    for (int i = 0; i < dim; i++)
                    {
                        cout << "vec " << i << ":\n" << basis[i] << endl;
                    }

                    return false;
                }

                tmpBasisV.push_back(newBasisVector);
            }

            basis = tmpBasisV;
        }

        // check if vectors are linearly independent
        /*
        int startWithDet = 0;
        int pos = 2;
        while (pos<dim)
        {

            Eigen::MatrixXf m(rows,pos);

            for (int i=0;i<pos;i++)
            {
                m.block(0,i,rows,1) = basis[i];
            }
            float d = m.determinant();
            if (d==0)
            {
                // if determinant = 0, the vectors are not linear independent
                startWithDet = pos;
                pos = dim+1; // end loop
                break;
            }

            pos++;
        }*/
        /*if (startWithDet!=dim)
        {

            std::vector< Eigen::VectorXf > tmpBasis;
            for (int j=0; j<startWithDet;j++)
            {
                tmpBasis.push_back(basis[j]);
            }
            for (int j=startWithDet;j<dim;j++)
            {
                Eigen::VectorXf newBasisVector;
                if (!randomLinearlyIndependentVector(tmpBasis,newBasisVector))
                {
                    VR_ERROR << "Could not find linearly independent basis vector?! " << endl;
                    for (int i=0;i<dim;i++)
                    {
                        cout << "vec " << i << ":\n" << basis[i] << endl;
                    }
                    return false;
                }
                tmpBasis.push_back(newBasisVector);
                basis[j] = newBasisVector;
            }
        }*/

        // check with which basis vector we must start

        // norm the first vector
        basis[0].normalize();
        int startWith = 0;
        int pos = 1;

        while (pos < dim)
        {
            for (int i = 0; i < pos; i++)
            {
                if (basis[i].dot(basis[pos]) != 0)
                {
                    startWith = pos;
                    pos = dim + 1; // end loop
                    break;
                }
            }

            pos++;
        }

        if (startWith == 0)
        {
            // nothing to do
            return true;
        }


        float scPr;

        for (int i = startWith; i < dim; i++) // each basis vector
        {
            // store original vector
            Eigen::Vector3f tmp = basis[i];

            for (int j = 0; j < i; j++) // subtract scalar products of former basises
            {
                // create scalar product
                scPr = tmp.dot(basis[j]);
                basis[i] -= scPr * basis[j];
                /*for (k=0;k<dim;k++) // each entry in vector
                {
                    basis[i][k] -= scPr*basis[j][k];
                }*/
            }

            basis[i].normalize();
        }

        return true;
    }

    bool VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::randomLinearlyIndependentVector(const std::vector< Eigen::VectorXf > basis, Eigen::VectorXf& storeResult)
    {
        VR_ASSERT_MESSAGE(basis.size() > 0, "need vectors");
        int dim = (int)basis[0].rows();
        VR_ASSERT_MESSAGE((int)basis.size() < dim, "Could not get a linearly independent vector, since basis already covers all dimensions.");

        for (int i = 1; i < (int)basis.size(); i++)
        {
            VR_ASSERT_MESSAGE((int)basis[i].rows() == (int)basis[0].rows(), "Length of basis vectors is wrong");
        }

        int loop = 0;
        Eigen::MatrixXf m(dim, basis.size() + 1);

        for (int i = 0; i < (int)basis.size(); i++)
        {
            m.block(0, i, dim, 1) = basis[i];
        }

        while (true)
        {
            storeResult = Eigen::VectorXf::Random(dim);

            if (storeResult.norm() > 1e-8)
            {
                storeResult.normalize();

                m.block(0, basis.size(), dim, 1) = storeResult;

                Eigen::FullPivLU<Eigen::MatrixXf> lu_decomp(m);

                if (lu_decomp.rank() == basis.size() + 1)
                {
                    return true;
                }
            }

            loop++;

            if (loop > 100)
            {
                VR_ERROR << "Could not determine independent vector, aborting after 100 tries" << endl;
                return false;
            }
        }
    }

    bool VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::ensureOrthonormalBasis(Eigen::Vector3f& x, Eigen::Vector3f& y, Eigen::Vector3f& z)
    {
        std::vector< Eigen::VectorXf > basis;
        basis.push_back(x);
        basis.push_back(y);
        basis.push_back(z);

        bool res = GramSchmidt(basis);

        if (!res)
        {
            return false;
        }

        x = basis[0];
        y = basis[1];
        z = basis[2];
        return true;
        /*
        const float minLength = 1e-10f;
        const float orthoCheck = 1e-3f;

        // check length
        float nx = x.norm();
        float ny = y.norm();
        float nz = z.norm();
        if (nx<minLength && ny<minLength && nz<minLength)
            return false;
        bool needX = (nx<minLength);
        bool needY = (ny<minLength);
        bool needZ = (nz<minLength);

        while (x.norm()<minLength)
            x.setRandom();
        while (y.norm()<minLength)
            y.setRandom();
        while (z.norm()<minLength)
            z.setRandom();

        x.normalize();
        y.normalize();
        z.normalize();

        if (needX)
        {
            x = y.cross(z);
            x.normalize();
        }
        if (needY)
        {
            y = z.cross(x);
            y.normalize();
        }
        if (needZ)
        {
            z = x.cross(y);
            z.normalize();
        }
        bool succ;
        int loop = 0;
        do
        {
            succ = true;
            // now check if vectors are orthogonal

            // z
            if (fabs(x.dot(z)) > orthoCheck)
            {
                z = x.cross(y);
                z.normalize();
                succ = false;
            }
            // y
            if (fabs(y.dot(z)) > orthoCheck)
            {
                y = z.cross(x);
                y.normalize();
                succ = false;
            }
            // x
            if (fabs(x.dot(y)) > orthoCheck)
            {
                x = y.cross(z);
                x.normalize();
                succ = false;
            }
            loop++;
            if (loop>10)
                return false;
        } while (!succ);

        return true;*/
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::eigen4f2axisangle(const Eigen::Matrix4f& m, Eigen::Vector3f& storeAxis, float& storeAngle)
    {
        Eigen::AngleAxis<float> aa(m.block<3, 3>(0, 0));
        storeAxis = aa.axis();
        storeAngle = aa.angle();
    }

    Eigen::Matrix3f VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::axisangle2eigen3f(const Eigen::Vector3f& axis, float angle)
    {
        Eigen::AngleAxis<float> aa(angle, axis);
        return aa.matrix();
    }

    Eigen::Matrix4f VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::axisangle2eigen4f(const Eigen::Vector3f& axis, float angle)
    {
        Eigen::Matrix4f res4 = Eigen::Matrix4f::Identity();
        res4.block(0, 0, 3, 3) =  axisangle2eigen3f(axis, angle);
        return res4;
    }

    float VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getAngle(const Quaternion& q)
    {
        float n = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

        if (n < 1e-10)
        {
            return 0.0f;
        }

        n = 1.0f / n;

        return (float)(2.0f * acosf(q.w * n));
    }

    float VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getAngle(const Eigen::Vector3f& v1, const Eigen::Vector3f& v2)
    {
        float res = v1.dot(v2);

        if (res < -1.0f)
        {
            res = -1.0f;
        }

        if (res > 1.0f)
        {
            res = 1.0f;
        }

        return acosf(res);
    }

    MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getDelta(const Quaternion& q1, const Quaternion& q2)
    {
        Quaternion q1I = getInverse(q1);
        return multiplyQuaternions(q2, q1I);
    }

    void VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getDelta(const Eigen::Matrix4f& m1, const Eigen::Matrix4f& m2, float& storeDetalPos, float& storeDeltaRot)
    {
        Eigen::Matrix4f deltaPose = m1.inverse() * m2;

        storeDetalPos = getTranslation(deltaPose).norm();
        Eigen::Vector3f tmp;
        eigen4f2axisangle(deltaPose, tmp, storeDeltaRot);

    }

    MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getInverse(const Quaternion& q)
    {
        float tmpQ = (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

        if (tmpQ == 0.0f)
        {
            tmpQ = FLT_MIN;
        }

        float Nq = 1.0f / tmpQ;
        Quaternion res = getConjugated(q);
        res.x *= Nq;
        res.y *= Nq;
        res.z *= Nq;
        res.w *= Nq;
        return res;
    }

    MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getConjugated(const Quaternion& q)
    {
        Quaternion res;
        res.x = q.x * (-1.0f);
        res.y = q.y * (-1.0f);
        res.z = q.z * (-1.0f);
        res.w = q.w;
        return res;
    }

    MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::multiplyQuaternions(const Quaternion& q1, const Quaternion& q2)
    {
        Quaternion res;
        res.x = q1.w * q2.x + q2.w * q1.x + q1.y * q2.z - q1.z * q2.y;
        res.y = q1.w * q2.y + q2.w * q1.y - q1.x * q2.z + q1.z * q2.x;
        res.z = q1.w * q2.z + q2.w * q1.z + q1.x * q2.y - q1.y * q2.x;
        res.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
        return res;
    }

    MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::eigen4f2quat(const Eigen::Matrix4f& m)
    {
        Eigen::Matrix3f m3 = m.block(0, 0, 3, 3);
        Eigen::Quaternionf q(m3);
        Quaternion res;
        res.x = q.x();
        res.y = q.y();
        res.z = q.z();
        res.w = q.w();
        return res;
    }

    MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::axisangle2quat(const Eigen::Vector3f& axis, float angle)
    {
        Eigen::Matrix4f m = axisangle2eigen4f(axis, angle);
        return eigen4f2quat(m);
    }

    float MathTools::getDot(const Quaternion& q1, const Quaternion& q2)
    {
        return q1.x * q2.x + q1.y * q2.y + q1.z * q2.z + q1.w * q2.w;
    }

    MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getMean(std::vector<MathTools::Quaternion> quaternions)
    {
        Quaternion res;

        if (quaternions.size() == 0)
        {
            return res;
        }

        res.x = res.y = res.z = res.w = 0;

        for (size_t i = 0; i < quaternions.size(); i++)
        {
            if (getDot(res, quaternions[i]) > 0)
            {
                res.x += quaternions[i].x;
                res.y += quaternions[i].y;
                res.z += quaternions[i].z;
                res.w += quaternions[i].w;
            }
            else
            {
                res.x += -quaternions[i].x;
                res.y += -quaternions[i].y;
                res.z += -quaternions[i].z;
                res.w += -quaternions[i].w;
            }
        }

        float mag = sqrtf(res.x * res.x + res.y * res.y + res.z * res.z + res.w * res.w);

        if (mag > 0.0001)
        {
            res.x /= mag;
            res.y /= mag;
            res.z /= mag;
            res.w /= mag;
        }
        else
        {
            res = quaternions[0];
        }

        return res;
    }

    MathTools::Quaternion VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::slerp(const MathTools::Quaternion& q1, const MathTools::Quaternion& q2, float alpha)
    {
        THROW_VR_EXCEPTION_IF(alpha < 0 || alpha > 1.0f, "Invalid alpha");
        Eigen::Quaternion<float> eq1(q1.w, q1.x, q1.y, q1.z);
        Eigen::Quaternion<float> eq2(q2.w, q2.x, q2.y, q2.z);
        Eigen::Quaternion<float> eq = eq1.slerp(alpha, eq2);
        MathTools::Quaternion res;
        res.x = eq.x();
        res.y = eq.z();
        res.z = eq.x();
        res.w = eq.w();
        return res;
    }


    Eigen::MatrixXf VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getBasisTransformation(const std::vector< Eigen::VectorXf >& basisSrc, const std::vector< Eigen::VectorXf >& basisDst)
    {
        THROW_VR_EXCEPTION_IF(basisSrc.size() == 0, "NULL size");
        THROW_VR_EXCEPTION_IF(basisSrc.size() != basisDst.size(), "Different size of basis vectors");
        int rows = basisSrc[0].rows();
        THROW_VR_EXCEPTION_IF(rows != basisDst[0].rows(), "Different row size of basis vectors");
        Eigen::MatrixXf mSrc(rows, basisSrc.size());
        Eigen::MatrixXf mDst(rows, basisDst.size());

        for (size_t i = 0; i < basisSrc.size(); i++)
        {
            THROW_VR_EXCEPTION_IF(rows != basisSrc[i].rows(), "Different row size of basis vectors");
            THROW_VR_EXCEPTION_IF(rows != basisDst[i].rows(), "Different row size of basis vectors");
            mSrc.block(0, i, rows, 1) = basisSrc[i];
            mDst.block(0, i, rows, 1) = basisDst[i];
        }

        return getBasisTransformation(mSrc, mDst);
    }

    Eigen::MatrixXf VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getBasisTransformation(const Eigen::MatrixXf& basisSrc, const Eigen::MatrixXf& basisDst)
    {
        THROW_VR_EXCEPTION_IF(basisSrc.rows() == 0 || basisSrc.cols() == 0, "NULL size");
        THROW_VR_EXCEPTION_IF(basisSrc.rows() != basisDst.rows(), "Different row sizes");
        THROW_VR_EXCEPTION_IF(basisSrc.cols() != basisDst.cols(), "Different col sizes");


        return basisDst.householderQr().solve(basisSrc);
    }

    MathTools::Plane VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getFloorPlane()
    {
        return Plane(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 1.0f));
    }

    MathTools::Line VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::intersectPlanes(const Plane& p1, const Plane& p2)
    {
        // algorithm taken from http://paulbourke.net/geometry/planeplane/

        float dotpr = p1.n.dot(p2.n);

        if (fabs(dotpr - 1.0f) < 1.e-6)
        {
            // planes are parallel
            MathTools::Line l(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));
            return l;
        }

        Eigen::Vector3f dir = p1.n.cross(p2.n);

        float d1 = p1.n.dot(p1.p);
        float d2 = p2.n.dot(p2.p);

        float det = p1.n.dot(p1.n) * p2.n.dot(p2.n) - p1.n.dot(p2.n) * p1.n.dot(p2.n);

        if (fabs(det) < 1e-9)
        {
            return MathTools::Line(Eigen::Vector3f(0, 0, 0), Eigen::Vector3f(0, 0, 0));
        }

        float c1 = (d1 * (p2.n.dot(p2.n)) - d2 * (p1.n.dot(p2.n))) / det;
        float c2 = (d2 * (p1.n.dot(p1.n)) - d1 * (p1.n.dot(p2.n))) / det;
        Eigen::Vector3f pos;
        pos = c1 * p1.n + c2 * p2.n;
        MathTools::Line l(pos, dir);
        // move pos so that we have a position that is next to plane point p1.p
        pos = nearestPointOnLine(l, p1.p);

        return MathTools::Line(pos, dir);
    }

    float VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::rad2deg(float rad)
    {
        static const float c = (float)(180.0 / M_PI);
        return rad * c;
    }

    float VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::deg2rad(float deg)
    {
        static const float c = (float)(M_PI / 180.0);
        return deg * c;
    }

    float VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getCartesianPoseDiff(const Eigen::Matrix4f& p1, const Eigen::Matrix4f& p2, float rotInfluence /*= 3.0f*/)
    {
        float tr = (p2.block(0, 3, 3, 1) - p1.block(0, 3, 3, 1)).norm();
        Quaternion q1 = eigen4f2quat(p1);
        Quaternion q2 = eigen4f2quat(p2);
        Quaternion d = getDelta(q1, q2);
        float ro = 180.0f - (d.w + 1.0f) * 90.0f;

        if (ro < 0)
        {
            if (ro < -0.001) // no output on rounding errors
            {
                VR_WARNING << "oops,calcCartesianPoseDiff: rotdist<0:" << ro << std::endl;
            }

            ro = fabs(ro);
        }

        if (ro > 180.0f)
        {
            VR_WARNING << "oops,calcCartesianPoseDiff: rotdist>180:" << ro << std::endl;
            ro = 180.0f;
        }

        return (tr + ro * rotInfluence);
    }

    MathTools::IntersectionResult VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::intersectSegmentPlane(const Segment& segment, const Plane& plane, Eigen::Vector3f& storeResult)
    {
        // check for same side
        if (onNormalPointingSide(segment.p0, plane) == onNormalPointingSide(segment.p1, plane))
        {
            return eNoIntersection;
        }

        Eigen::Vector3f u = segment.p1 - segment.p0;
        Eigen::Vector3f w = segment.p0 - plane.p;

        float     D = plane.n.dot(u);// dot(Pn.n, u);
        float     N = -plane.n.dot(w);//-dot(Pn.n, w);

        // check for parallel
        if (fabs(D) < MIN_TO_ZERO)
        {
            return eNoIntersection;
            /*if (N == 0)                     // segment lies in plane
                return 2;
            else
                return 0;                   // no intersection*/
        }

        float sI = N / D;

        if (sI < 0 || sI > 1)
        {
            return eNoIntersection;    // no intersection (should be handled above, just to be sure)
        }

        storeResult = segment.p0 + sI * u;                 // compute segment intersect point
        return eIntersection;
    }

    MathTools::IntersectionResult VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::intersectOOBBPlane(const OOBB& oobb, const Plane& plane, std::vector<Eigen::Vector3f>& storeResult)
    {
        std::vector<MathTools::Segment> s = oobb.getSegments();
        VR_ASSERT(s.size() == 12);

        Eigen::Vector3f res;
        std::vector<Eigen::Vector3f> intersectionPoints;

        for (int i = 0; i < 12; i++)
        {
            if (intersectSegmentPlane(s[i], plane, res) == eIntersection)
            {
                // check for double entries
                bool ok = true;

                for (size_t j = 0; j < intersectionPoints.size(); j++)
                {
                    if ((intersectionPoints[j] - res).norm() <= 1e-6)
                    {
                        ok = false;
                    }
                }

                if (ok)
                {
                    intersectionPoints.push_back(res);
                }
            }
        }

        if (intersectionPoints.size() < 3)
        {
            return eNoIntersection;
        }

        storeResult = intersectionPoints;
        return eIntersection;
    }

    Eigen::VectorXf VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getPermutation(const Eigen::VectorXf& inputA, const Eigen::VectorXf& inputB, unsigned int i)
    {
        VR_ASSERT(inputA.rows() == inputB.rows());
        Eigen::VectorXf result = inputA;

        for (int j = 0; j < inputA.rows(); j++)
        {
            if (i % 2 == 0)
            {
                result(j) = inputB(j);
            }

            i = i >> 1;
        }

        return result;
    }

    Eigen::Vector3f VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::toPosition(const SphericalCoord& sc)
    {
        Eigen::Vector3f res;

        res(0) = sc.r * sin(sc.theta) * cos(sc.phi);
        res(1) = sc.r * sin(sc.theta) * sin(sc.phi);
        res(2) = sc.r * cos(sc.theta);
        return res;
    }

    VirtualRobot::MathTools::SphericalCoord VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::toSphericalCoords(const Eigen::Vector3f& pos)
    {
        VirtualRobot::MathTools::SphericalCoord res;
        res.r = pos.norm();

        if (res.r < 1e-8)
        {
            res.phi = 0;
            res.theta = 0;
            return res;
        }

        res.phi = atan2(pos(1), pos(0));
        res.theta = acos(pos(2) / res.r);
        return res;
    }

    int VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::pow_int(int a, int b)
    {
        int powI = 1;

        for (int j = 0; j < b; j++)
        {
            powI *= a;
        }

        return powI;

    }

    Eigen::MatrixXf VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getPseudoInverse(const Eigen::MatrixXf& m, float tol /*= 1e-5f*/)
    {
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXf U = svd.matrixU();
        Eigen::MatrixXf V = svd.matrixV();
        Eigen::VectorXf sv = svd.singularValues();

        for (int i = 0; i < sv.rows(); i++)
            if (sv(i) > tol)
            {
                sv(i) = 1.0f / sv(i);
            }
            else
            {
                sv(i) = 0;
            }

        return (V * sv.asDiagonal() * U.transpose());
    }


    Eigen::MatrixXf VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getPseudoInverseDamped(const Eigen::MatrixXf& m, float lambda /*= 1.0f*/)
    {
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXf U = svd.matrixU();
        Eigen::MatrixXf V = svd.matrixV();
        Eigen::VectorXf sv = svd.singularValues();

        for (int i = 0; i < sv.rows(); i++)
        {
            sv(i) = sv(i) / (sv(i) * sv(i) + lambda * lambda);
        }

        return (V * sv.asDiagonal() * U.transpose());
    }

    Eigen::Vector2f VIRTUAL_ROBOT_IMPORT_EXPORT MathTools::getConvexHullCenter(ConvexHull2DPtr ch)
    {
        Eigen::Vector2f c;
        c << 0, 0;

        if (!ch || ch->vertices.size() == 0)
        {
            VR_WARNING << "Null data..." << endl;
            return c;
        }

        Eigen::Vector2f minPos = ch->vertices[0];
        Eigen::Vector2f maxPos = ch->vertices[0];

        for (size_t i = 1; i < ch->vertices.size(); i++)
        {
            if (minPos[0] > ch->vertices[i][0])
            {
                minPos[0] = ch->vertices[i][0];
            }

            if (minPos[1] > ch->vertices[i][1])
            {
                minPos[1] = ch->vertices[i][1];
            }

            if (maxPos[0] < ch->vertices[i][0])
            {
                maxPos[0] = ch->vertices[i][0];
            }

            if (maxPos[1] < ch->vertices[i][1])
            {
                maxPos[1] = ch->vertices[i][1];
            }
        }

        c = (minPos + maxPos) * 0.5f;
        return c;
    }



} // namespace

