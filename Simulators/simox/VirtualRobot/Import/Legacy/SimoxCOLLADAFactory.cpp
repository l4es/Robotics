

#include "SimoxCOLLADAFactory.h"
#include "../COLLADA/ColladaIO.h"


namespace VirtualRobot
{

    SimoxCOLLADAFactory::SimoxCOLLADAFactory()
    {
    }


    SimoxCOLLADAFactory::~SimoxCOLLADAFactory()
    {
    }


    RobotPtr SimoxCOLLADAFactory::loadFromFile(const std::string& filename, RobotIO::RobotDescription loadMode)
    {
        RobotPtr robot;

        //loadMode is currently ignored
        try
        {
            robot = ColladaIO::loadRobot(filename);
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
    RobotImporterFactory::SubClassRegistry SimoxCOLLADAFactory::registry(SimoxCOLLADAFactory::getName(), &SimoxCOLLADAFactory::createInstance);


    /**
     * \return "SimoxCOLLADA"
     */
    std::string SimoxCOLLADAFactory::getName()
    {
        return "SimoxCOLLADA";
    }


    /**
     * \return new instance of SimoxCOLLADAFactory.
     */
    boost::shared_ptr<RobotImporterFactory> SimoxCOLLADAFactory::createInstance(void*)
    {
        boost::shared_ptr<SimoxCOLLADAFactory> COLLADAFactory(new SimoxCOLLADAFactory());
        return COLLADAFactory;
    }

} // namespace VirtualRobot
