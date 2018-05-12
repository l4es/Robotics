
#include "Grasp.h"
#include "../RobotConfig.h"
#include "..//Robot.h"
#include "../VirtualRobotException.h"
#include <iomanip>

namespace VirtualRobot
{


    Grasp::Grasp(const std::string& name, const std::string& robotType, const std::string& eef, const Eigen::Matrix4f& poseInTCPCoordSystem, const std::string& creation, float quality, const std::string& eefPreshape)
        : name(name), robotType(robotType), eef(eef), poseTcp(poseInTCPCoordSystem), creation(creation), quality(quality), preshape(eefPreshape)
    {
    }

    Grasp::~Grasp()
    {
    }

    void Grasp::print(bool printDecoration /*= true*/)
    {
        if (printDecoration)
        {
            cout << "**** Grasp ****" << endl;
            cout << " * Robot type: " << robotType << endl;
            cout << " * End Effector: " << eef << endl;
            cout << " * EEF Preshape: " << preshape << endl;
        }

        cout << " * Name: " << name << endl;
        cout << " * Creation Method: " << creation << endl;
        cout << " * Quality: " << quality << endl;
        {
            // scope
            std::ostringstream sos;
            sos << std::setiosflags(std::ios::fixed);
            sos << " * Pose in EEF-TCP coordinate system:" << endl << poseTcp << endl;
            cout << sos.str() << endl;
        } // scope

        if (printDecoration)
        {
            cout << endl;
        }
    }

    std::string Grasp::getRobotType()
    {
        return robotType;
    }

    std::string Grasp::getEefName()
    {
        return eef;
    }

    Eigen::Matrix4f Grasp::getTargetPoseGlobal(RobotPtr robot)
    {
        THROW_VR_EXCEPTION_IF(!robot, "Null data");
        THROW_VR_EXCEPTION_IF(robot->getType() != robotType, "Robot types are not compatible: " << robot->getType() << " != " << robotType);
        EndEffectorPtr eefPtr = robot->getEndEffector(eef);

        if (!eefPtr)
        {
            VR_ERROR << "No EndEffector with name " << eef << " stored in robot " << robot->getName() << endl;
            return Eigen::Matrix4f::Identity();
        }

        RobotNodePtr tcpNode = eefPtr->getTcp();

        if (!tcpNode)
        {
            VR_ERROR << "No tcp with name " << eefPtr->getTcpName() << " in EndEffector " << eef << " in robot " << robot->getName() << endl;
            return Eigen::Matrix4f::Identity();
        }

        return tcpNode->toGlobalCoordinateSystem(poseTcp);
    }

    std::string Grasp::getName()
    {
        return name;
    }

    std::string Grasp::getPreshapeName()
    {
        return preshape;
    }

    Eigen::Matrix4f Grasp::getTransformation()
    {
        return poseTcp;
    }

    void Grasp::setName(const std::string& name)
    {
        this->name = name;
    }

    std::string Grasp::toXML(int tabs)
    {
        std::stringstream ss;
        std::string t;

        for (int i = 0; i < tabs; i++)
        {
            t += "\t";
        }

        std::string tt = t;
        tt += "\t";
        std::string ttt = tt;
        ttt += "\t";
        ss << t << "<Grasp name='" << name << "' quality='" << quality << "' Creation='" << creation;

        if (preshape.empty())
        {
            ss << "'>\n";
        }
        else
        {
            ss << "' Preshape='" << preshape << "'>\n";
        }

        ss << tt << "<Transform>\n";
        ss << MathTools::getTransformXMLString(poseTcp, ttt);
        ss << tt << "</Transform>\n";

        if (eefConfiguration.size() > 0)
        {
            ss << RobotConfig::createXMLString(eefConfiguration, name, tabs + 1);
        }

        ss << t << "</Grasp>\n";

        return ss.str();
    }

    void Grasp::setTransformation(const Eigen::Matrix4f& tcp2Object)
    {
        poseTcp = tcp2Object;
    }

    Eigen::Matrix4f Grasp::getTcpPoseGlobal(const Eigen::Matrix4f& objectPose)
    {
        Eigen::Matrix4f result = objectPose * poseTcp.inverse();
        return result;
    }

    Eigen::Matrix4f Grasp::getObjectTargetPoseGlobal(const Eigen::Matrix4f& graspingPose)
    {
        Eigen::Matrix4f result = graspingPose * poseTcp;
        return result;
    }

    VirtualRobot::GraspPtr Grasp::clone()
    {
        GraspPtr result(new Grasp(name, robotType, eef, poseTcp, creation, quality, preshape));
        result->setConfiguration(eefConfiguration);
        return result;
    }

    void Grasp::setConfiguration(std::map< std::string, float >& c)
    {
        eefConfiguration = c;
    }

    std::map< std::string, float > Grasp::getConfiguration()
    {
        return eefConfiguration;
    }

    void Grasp::setPreshape(const std::string& preshapeName)
    {
        preshape = preshapeName;
    }

    float Grasp::getQuality()
    {
        return quality;
    }

    std::string Grasp::getCreationMethod()
    {
        return creation;
    }


} //  namespace


