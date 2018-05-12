#include "BoundingBox.h"

namespace VirtualRobot
{

    BoundingBox::BoundingBox()
    {
        min = max = Eigen::Vector3f::Zero();
    }

    bool BoundingBox::planeGoesThrough(const MathTools::Plane& p)
    {
        bool left = false;
        bool right = false;

        std::vector < Eigen::Vector3f > points = getPoints();

        for (int i = 0; i < 8; i++)
            if (MathTools::onNormalPointingSide(points[i], p))
            {
                left = true;
            }
            else
            {
                right = true;
            }

        if (!left || !right)
        {
            return false;
        }

        return true;
    }


    BoundingBox::BoundingBox(const std::vector< Eigen::Vector3f >& p)
    {
        if (p.size() == 0)
        {
            min = max = Eigen::Vector3f::Zero();
        }
        else
        {
            min = p[0];
            max = p[0];
            addPoints(p);
        }
    }

    std::vector <Eigen::Vector3f> BoundingBox::getPoints() const
    {
        std::vector < Eigen::Vector3f > points;

        points.push_back(Eigen::Vector3f(min(0), min(1), min(2)));
        points.push_back(Eigen::Vector3f(min(0), min(1), max(2)));
        points.push_back(Eigen::Vector3f(min(0), max(1), min(2)));
        points.push_back(Eigen::Vector3f(min(0), max(1), max(2)));

        points.push_back(Eigen::Vector3f(max(0), min(1), min(2)));
        points.push_back(Eigen::Vector3f(max(0), min(1), max(2)));
        points.push_back(Eigen::Vector3f(max(0), max(1), min(2)));
        points.push_back(Eigen::Vector3f(max(0), max(1), max(2)));

        return points;
    }

    void BoundingBox::print()
    {
        cout << "* Bounding Box\n";
        std::streamsize pr = cout.precision(2);
        cout << "** min <" << min(0) << "," << min(1) << "," << min(2) << ">\n";
        cout << "** max <" << max(0) << "," << max(1) << "," << max(2) << ">\n";
        cout.precision(pr);

    }

    void BoundingBox::addPoints(const std::vector < Eigen::Vector3f >& p)
    {
        for (size_t i = 0; i < p.size(); i++)
        {
            addPoint(p[i]);
        }
    }

    void BoundingBox::addPoints(const BoundingBox& bbox)
    {
        std::vector <Eigen::Vector3f> p = bbox.getPoints();
        addPoints(p);
    }

    void BoundingBox::addPoint(const Eigen::Vector3f& p)
    {
        for (int j = 0; j < 3; j++)
        {
            if (p(j) < min(j))
            {
                min(j) = p(j);
            }

            if (p(j) > max(j))
            {
                max(j) = p(j);
            }
        }
    }

    Eigen::Vector3f BoundingBox::getMax() const
    {
        return max;
    }

    Eigen::Vector3f BoundingBox::getMin() const
    {
        return min;
    }


    void BoundingBox::clear()
    {
        min.setZero();
        max.setZero();
    }

    void BoundingBox::transform(Eigen::Matrix4f& pose)
    {
        Eigen::Vector3f result[8];
        std::vector<Eigen::Vector3f> result3;
        result[0] << min(0), min(1), min(2);
        result[1] << max(0), min(1), min(2);
        result[2] << min(0), max(1), min(2);
        result[3] << max(0), max(1), min(2);
        result[4] << min(0), min(1), max(2);
        result[5] << max(0), min(1), max(2);
        result[6] << min(0), max(1), max(2);
        result[7] << max(0), max(1), max(2);
        Eigen::Matrix4f m;

        for (int i = 0; i < 8; i++)
        {
            m.setIdentity();
            m.block(0, 3, 3, 1) = result[i];
            m = pose * m;
            result3.push_back(m.block(0, 3, 3, 1));
        }

        // now find min max values
        min << FLT_MAX, FLT_MAX, FLT_MAX;
        max << -FLT_MAX, -FLT_MAX, -FLT_MAX;

        for (int i = 0; i < 8; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                if (result3[i](j) < min(j))
                {
                    min(j) = result3[i](j);
                }

                if (result3[i](j) > max(j))
                {
                    max(j) = result3[i](j);
                }
            }
        }
    }

    void BoundingBox::scale(Eigen::Vector3f& scaleFactor)
    {
        for (int i = 0; i < 3; i++)
        {
            min(i) *= scaleFactor(i);
            max(i) *= scaleFactor(i);
        }
    }


}
