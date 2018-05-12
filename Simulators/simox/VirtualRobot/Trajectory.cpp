#include "Trajectory.h"
#include "Robot.h"
#include "VirtualRobotException.h"
#include <iostream>
#include <sstream>

namespace VirtualRobot
{

    Trajectory::Trajectory(RobotNodeSetPtr rns, const std::string& name)
        : rns(rns), name(name)
    {
        THROW_VR_EXCEPTION_IF(!rns, "Need a rns defined...");
        dimension = rns->getSize();
        THROW_VR_EXCEPTION_IF(dimension == 0, "No joints defined in RNS...");
    }

    Trajectory::~Trajectory()
    {
        reset();
    }

    void Trajectory::reset()
    {
        path.clear();
    }

    unsigned int Trajectory::getDimension() const
    {
        return dimension;
    }


    void Trajectory::addPoint(const Eigen::VectorXf& c)
    {
        VR_ASSERT(c.rows() == dimension);
        path.push_back(c);
    }

    unsigned int Trajectory::getNrOfPoints() const
    {
        return (unsigned int)path.size();
    }


    Eigen::VectorXf Trajectory::getPoint(unsigned int nr) const
    {
        if (nr >= path.size())
        {
            VR_ERROR << nr << " >= " << (unsigned int)path.size() << std::endl;

            if (path.size() > 0)
            {
                return path[path.size() - 1];
            }
            else
            {
                Eigen::VectorXf x(dimension);
                x.setZero(dimension);
                return x;
            }
        }

        return path[nr];
    }

    TrajectoryPtr Trajectory::clone() const
    {
        TrajectoryPtr res(new Trajectory(rns, name));

        for (unsigned int i = 0; i < getNrOfPoints(); i++)
        {
            res->addPoint(getPoint(i));
        }

        return res;
    }


    TrajectoryPtr Trajectory::createSubPath(unsigned int startIndex, unsigned int endIndex) const
    {
        if (startIndex >= getNrOfPoints() || endIndex >= getNrOfPoints())
        {
            VR_ERROR << " wrong start or end pos" << std::endl;
            return TrajectoryPtr();
        }

        TrajectoryPtr res(new Trajectory(rns, name));

        for (unsigned int i = startIndex; i <= endIndex; i++)
        {
            res->addPoint(getPoint(i));
        }

        return res;
    }



    bool Trajectory::getPoints(unsigned int start, unsigned int end , std::vector<Eigen::VectorXf>& storePosList) const
    {
        if (start > end || end >= path.size())
        {
            VR_ERROR << "Trajectory::getPoints: wrong start or end.." << std::endl;
            return false;
        }

        unsigned int i = start;
        Eigen::VectorXf  data;

        while (i <= end)
        {
            data = getPoint(i);
            storePosList.push_back(data);
            i++;
        }

        return true;
    }


    void Trajectory::erasePosition(unsigned int pos)
    {
        if (pos >= path.size())
        {
            VR_ERROR << "Trajectory::erasePosition: pos not valid ?!" << std::endl;
            return;
        }

        std::vector<Eigen::VectorXf>::iterator iter = path.begin();
        iter += pos;

        path.erase(iter);
    }


    unsigned int Trajectory::removePositions(unsigned int startPos, unsigned int endPos)
    {
        if (startPos >= path.size() || endPos >= path.size())
        {
            VR_ERROR << "Trajectory::removePositions: pos not valid ?!" << std::endl;
            return 0;
        }

        if (startPos > endPos)
        {
            return 0;
        }

        std::vector<Eigen::VectorXf>::iterator iter = path.begin();
        iter += startPos;
        unsigned int result = 0;

        for (unsigned int i = startPos; i <= endPos; i++)
        {
            if (iter == path.end())
            {
                VR_ERROR << "Internal error in Trajectory::removePositions..." << std::endl;
                return result;
            }

            //delete *iter;
            iter = path.erase(iter);
            result++;
        }

        return result;
    }

    // inserts at position before pos
    void Trajectory::insertPosition(unsigned int pos, const Eigen::VectorXf& c)
    {
        if (pos > path.size())
        {
            std::cout << "Trajectory::insertPosition: pos not valid ?!" << std::endl;
            return;
        }

        // copy config and insert it
        std::vector<Eigen::VectorXf>::iterator iter = path.begin();
        iter += pos;

        path.insert(iter,  c);
    }

    void Trajectory::insertPosition(unsigned int pos, std::vector<Eigen::VectorXf>& newConfigurations)
    {
        std::vector<Eigen::VectorXf>::iterator iter = newConfigurations.begin();

        while (iter != newConfigurations.end())
        {
            insertPosition(pos, *iter);
            pos++;
            iter++;
        }
    }

    void Trajectory::insertTrajectory(unsigned int pos, TrajectoryPtr trajectoryToInsert)
    {
        if (!trajectoryToInsert)
        {
            VR_ERROR << "null data" << endl;
            return;
        }

        int i = trajectoryToInsert->getNrOfPoints() - 1;

        while (i >= 0)
        {
            insertPosition(pos, trajectoryToInsert->getPoint(i));
            i--;
        }
    }


    // returns position on path for time t (0<=t<=1)
    void Trajectory::interpolate(float t, Eigen::VectorXf& storePathPos, int* storeIndex /*= NULL*/) const
    {
        storePathPos.resize(dimension);

        if (t < 0 || t > 1.0f)
        {
            // check for rounding errors
            if (t < -0.000000001 || t > 1.000001f)
            {
                VR_ERROR << " need t value between 0 and 1... (" << t << ")" << std::endl;
            }

            if (t < 0)
            {
                t = 0.0f;
            }

            if (t > 1.0f)
            {
                t = 1.0f;
            }
        }

        if (getNrOfPoints() == 0)
        {
            VR_WARNING << " no Path.." << std::endl;
            return;
        }

        if (t == 0.0f)
        {
            storePathPos = getPoint(0);

            if (storeIndex != NULL)
            {
                *storeIndex = 0;
            }

            return;
        }
        else if (t == 1.0f)
        {
            storePathPos = getPoint(getNrOfPoints() - 1);

            if (storeIndex != NULL)
            {
                *storeIndex = (int)path.size() - 1;
            }

            return;
        }


        float l = getLength();
        float wantedLength = l * t;
        float actLength = 0.0f;
        unsigned int startIndex = 0;
        Eigen::VectorXf c1 = getPoint(startIndex);
        Eigen::VectorXf c2 = c1;
        float lastLength = 0.0f;

        // search path segment for wantedLength
        while (actLength < wantedLength && startIndex < getNrOfPoints() - 1)
        {
            c1 = c2;
            startIndex++;
            c2 = getPoint(startIndex);
            lastLength = (c2 - c1).norm();
            //lastLength = MathHelpers::calcDistRotational(c1, c2, dimension, m_rotationalDimension);
            actLength += lastLength;
        }

        startIndex--;
        actLength -= lastLength;

        // segment starts with startIndex
        float restLength = wantedLength - actLength;

        float factor = 0.0f;

        if (lastLength > 0)
        {
            factor = restLength / lastLength;
        }

        if (factor > 1.0f)
        {
            // ignore rounding errors
            factor = 1.0f;
        }

        storePathPos = c1 + (c2 - c1) * factor; // storePos = startPos + factor*segment

        if (storeIndex != NULL)
        {
            *storeIndex = startIndex;
        }

    }

    void Trajectory::reverse()
    {
        std::reverse(path.begin(), path.end());
    }

    float Trajectory::getLength() const
    {
        float pathLength = 0.0f;
        Eigen::VectorXf c1, c2;
        float l;

        for (int i = 0; i < (int)getNrOfPoints() - 1; i++)
        {
            c1 = path[i];
            c2 = path[i + 1];

            l = (c2 - c1).norm();
            pathLength += l;
        }

        return pathLength;
    }


    const std::vector <Eigen::VectorXf>& Trajectory::getData() const
    {
        return path;
    }

    std::vector<Eigen::Matrix4f > Trajectory::createWorkspaceTrajectory(VirtualRobot::RobotNodePtr r)
    {
        VR_ASSERT(rns);

        if (!r)
        {
            r = rns->getTCP();
        }

        VR_ASSERT(r);

        RobotPtr robot = rns->getRobot();
        VR_ASSERT(robot);

        Eigen::VectorXf jv;
        rns->getJointValues(jv);

        std::vector<Eigen::Matrix4f > result;

        for (size_t i = 0; i < path.size(); i++)
        {
            // get tcp coords:
            robot->setJointValues(rns, path[i]);
            Eigen::Matrix4f m;
            result.push_back(r->getGlobalPose());
        }

        robot->setJointValues(rns, jv);
        return result;
    }

    std::string Trajectory::toXML(int tabs) const
    {
        std::stringstream ss;
        std::string tab = "";

        for (int i = 0; i < tabs; i++)
        {
            tab += "\t";
        }

        RobotPtr robot = rns->getRobot();

        THROW_VR_EXCEPTION_IF((!robot || !rns), "Need a valid robot and rns");


        ss << tab << "<Trajectory Robot='" << robot->getType() << "' RobotNodeSet='" << rns->getName() << "' name='" << name << "' dim='" << dimension << "'>\n";

        for (unsigned int i = 0; i < path.size(); i++)
        {
            ss << tab << "\t<Point id='" << i << "'>\n";
            Eigen::VectorXf c = path[i];

            for (unsigned int k = 0; k < dimension; k++)
            {
                ss << tab << "\t\t<c value='" << c[k] << "'/>\n";
            }

            ss << tab << "\t</Point>\n";
        }

        ss << tab << "</Trajectory>\n";
        return ss.str();
    }

    void Trajectory::print() const
    {
        std::string s = toXML(0);
        VR_INFO << s << endl;
    }

    std::string Trajectory::getName() const
    {
        return name;
    }

    VirtualRobot::RobotNodeSetPtr Trajectory::getRobotNodeSet()
    {
        return rns;
    }

    VisualizationNodePtr Trajectory::getVisualization(std::string visualizationFactoryName)
    {
        VisualizationFactoryPtr visualizationFactory;

        if (visualizationFactoryName.empty())
        {
            visualizationFactory = VisualizationFactory::first(NULL);
        }
        else
        {
            visualizationFactory = VisualizationFactory::fromName(visualizationFactoryName, NULL);
        }

        if (!visualizationFactory)
        {
            VR_ERROR << "Could not create factory for visu type " << visualizationFactoryName << endl;
            return VisualizationNodePtr();
        }

        return visualizationFactory->createTrajectory(shared_from_this());
    }

    std::string Trajectory::getRobotName() const
    {
        return rns->getRobot()->getName();
    }
    /*
    void Trajectory::apply( float t )
    {
        if (!rns)
            return;
        RobotPtr robot = rns->getRobot();
        Eigen::VectorXf c;
        interpolate(t,c);
        robot->setJointValues(rns,c);
    }
    */
}
