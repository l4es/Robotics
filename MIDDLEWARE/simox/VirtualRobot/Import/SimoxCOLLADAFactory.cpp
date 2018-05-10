

#include "SimoxCOLLADAFactory.h"
#include "COLLADA-light/ColladaSimox.h"


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

        try
        {
            Collada::ColladaSimoxRobot colladaRobot(1000.0f);
            colladaRobot.parse(filename);
            colladaRobot.initialize();
            robot = colladaRobot.getSimoxRobot();
        }
        catch (VirtualRobotException& e)
        {
            cout << " ERROR while creating robot (exception)" << endl;
            cout << e.what();
            return robot;
        }

        if (!robot)
        {
            VR_ERROR << " ERROR while loading robot from file:" << filename << endl;
        }

        return robot;
    }

    std::string SimoxCOLLADAFactory::getFileExtension()
    {
        return std::string("dae");
    }

    std::string SimoxCOLLADAFactory::getFileFilter()
    {
        return std::string("COLLADA (*.dae)");
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
