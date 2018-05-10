

#include "SimoxXMLFactory.h"
#include "../XML/RobotIO.h"
#include "../XML/rapidxml.hpp"


namespace VirtualRobot
{

    SimoxXMLFactory::SimoxXMLFactory()
    {
    }


    SimoxXMLFactory::~SimoxXMLFactory()
    {
    }


    RobotPtr SimoxXMLFactory::loadFromFile(const std::string& filename, RobotIO::RobotDescription loadMode)
    {
        RobotPtr robot;

        try
        {
            robot = RobotIO::loadRobot(filename, loadMode);
        }
        catch (VirtualRobotException& e)
        {
            VR_ERROR << " ERROR while loading robot from file:" << filename << endl;
            VR_ERROR << e.what();
            return robot;
        }

        if (!robot)
        {
            VR_ERROR << " ERROR while loading robot from file:" << filename << endl;
        }

        return robot;
    }

    /**
     * register this class in the super class factory
     */
    RobotImporterFactory::SubClassRegistry SimoxXMLFactory::registry(SimoxXMLFactory::getName(), &SimoxXMLFactory::createInstance);


    /**
     * \return "SimoxXML"
     */
    std::string SimoxXMLFactory::getName()
    {
        return "SimoxXML";
    }


    /**
     * \return new instance of SimoxXMLFactory.
     */
    boost::shared_ptr<RobotImporterFactory> SimoxXMLFactory::createInstance(void*)
    {
        boost::shared_ptr<SimoxXMLFactory> xmlFactory(new SimoxXMLFactory());
        return xmlFactory;
    }

    std::string SimoxXMLFactory::getFileFilter()
    {
        return std::string("Simox XML (*.xml)");
    }

    std::string VirtualRobot::SimoxXMLFactory::getFileExtension()
    {
        return std::string("xml");
    }

} // namespace VirtualRobot
