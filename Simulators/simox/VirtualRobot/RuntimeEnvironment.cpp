#ifdef WIN32
#   pragma warning (disable:4275) // non dll-interface class 'std::logic_error' used as base for dll-interface class 'boost::program_options::error'
#   pragma warning (disable:4251) // class 'std::vector<_Ty>' needs to have dll-interface to be used by clients of class 'boost::program_options::ambiguous_option'
#   pragma warning (disable:4996) // warning on insecure char* usage, that arises when using boost::split
#endif

#include "RuntimeEnvironment.h"
#include "VirtualRobotException.h"
#include "Visualization/VisualizationFactory.h"


namespace VirtualRobot
{
    std::vector< std::string > RuntimeEnvironment::processKeys;
    std::vector< std::string > RuntimeEnvironment::unrecognizedOptions;
    std::vector< std::string > RuntimeEnvironment::dataPaths;
    std::map< std::string, std::string > RuntimeEnvironment::keyValues;
    bool RuntimeEnvironment::pathInitialized = false;

    void RuntimeEnvironment::init()
    {
        if (!pathInitialized)
        {
            pathInitialized = true;
            bool pathFound = false;
            char* simox_data_path = getenv("SIMOX_DATA_PATH");

            if (simox_data_path)
            {
                pathFound = addDataPath(std::string(simox_data_path), true);
            }

            char* vr_data_path = getenv("VIRTUAL_ROBOT_DATA_PATH");

            if (vr_data_path)
            {
                pathFound = addDataPath(std::string(vr_data_path), true);
            }

            if (!pathFound)
            {
                // test for Simox_DIR
                simox_data_path = getenv("Simox_DIR");

                if (simox_data_path)
                {
                    std::string sd(simox_data_path);
                    sd += std::string("/data");
                    pathFound = addDataPath(sd, true);

                    if (!pathFound)
                    {
                        std::string sd(simox_data_path);
                        sd += std::string("/VirtualRobot/data");
                        pathFound = addDataPath(sd, true);
                    }

                    if (!pathFound)
                    {
                        std::string sd(simox_data_path);
                        sd += std::string("../VirtualRobot/data");
                        pathFound = addDataPath(sd, true);
                    }
                }
            }

#ifdef Simox_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(Simox_DATA_PATH), true);
#endif
#ifdef VirtualRobot_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(VirtualRobot_DATA_PATH), true);
#endif
#ifdef VirtualRobot_SRC_DATA_PATH
            pathFound = pathFound | addDataPath(std::string(VirtualRobot_SRC_DATA_PATH), true);
#endif

            // check standard linux install path
            if (!pathFound)
            {
                pathFound = addDataPath(std::string("/usr/local/data"), true);
            }

            if (!pathFound)
            {
                pathFound = addDataPath(std::string("/usr/data"), true);
            }

            // last chance, check for inbuild paths
            if (!pathFound)
            {
                boost::filesystem::path p(boost::filesystem::current_path());
                boost::filesystem::path p1 = boost::filesystem::operator/(p, "../VirtualRobot/data");
                boost::filesystem::path p2 = boost::filesystem::operator/(p, "../../VirtualRobot/data");
                boost::filesystem::path p3 = boost::filesystem::operator/(p, "../../../VirtualRobot/data");
                boost::filesystem::path p4 = boost::filesystem::operator/(p, "../../../../VirtualRobot/data");
                pathFound = pathFound | addDataPath(p1.string(), true);
                pathFound = pathFound | addDataPath(p2.string(), true);
                pathFound = pathFound | addDataPath(p3.string(), true);
                pathFound = pathFound | addDataPath(p4.string(), true);
            }
        }
    }

    bool RuntimeEnvironment::getDataFileAbsolute(std::string& fileName)
    {
        if (!pathInitialized)
        {
            init();
        }

        boost::filesystem::path fn(fileName);

        try
        {
            // first check current path
            if (boost::filesystem::exists(fn) && boost::filesystem::is_regular_file(fn))
            {
                fileName = fn.string();
                return true;
            }
        }
        catch (...)//const boost::filesystem::filesystem_error& /*ex*/)
        {
            //cout << ex.what() << '\n';
            // silently skip this error (e.g. device not ready, permission denied etc)
        }

        for (size_t i = 0; i < dataPaths.size(); i++)
        {
            boost::filesystem::path p(dataPaths[i]);

            boost::filesystem::path fnComplete = boost::filesystem::operator/(p, fn);

            try
            {
                if (boost::filesystem::exists(fnComplete) && boost::filesystem::is_regular_file(fnComplete))
                {
                    // check for permissions (todo)
                    //boost::filesystem::file_status s = boost::filesystem::status(fnComplete);
                    //printf("%o\n",s.permissions());

                    fileName = fnComplete.string();
                    return true;
                }
            }
            catch (...)//const boost::filesystem::filesystem_error& ex)
            {
                //cout << "EX:" << ex.what() << '\n';
                // silently skip this error (e.g. device not ready, permission denied etc)
            }
        }

        return false;
    }


    void RuntimeEnvironment::processCommandLine(int argc, char* argv[])
    {
        if (!pathInitialized)
        {
            init();
        }

        // Declare the supported options.
        boost::program_options::options_description desc("Simox runtime options");
        desc.add_options()
        ("help", "Simox command line parser: Set options with '--key value'\n")
        ("data-path", boost::program_options::value< std::vector< std::string > >()->composing(), "Set data path. Multiple data paths are allowed.")
        ;

        for (size_t i = 0; i < processKeys.size(); i++)
            desc.add_options()
            (processKeys[i].c_str(), boost::program_options::value< std::vector< std::string > >(), processKeys[i].c_str())
            ;

        boost::program_options::parsed_options parsed =
            boost::program_options::command_line_parser(argc, argv).options(desc).allow_unregistered().run();

        boost::program_options::variables_map vm;
        //boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
        boost::program_options::store(parsed, vm);
        boost::program_options::notify(vm);

        // process data-path entries
        if (vm.count("data-path"))
        {
            //VR_INFO << "Data paths are: " << endl;
            std::vector< std::string > dp = vm["data-path"].as< std::vector< std::string > >();

            for (size_t i = 0; i < dp.size(); i++)
            {
                addDataPath(dp[i]);
                //VR_INFO << dp[i] << "\n";
            }
        }

        // process generic keys
        for (size_t i = 0; i < processKeys.size(); i++)
        {
            if (vm.count(processKeys[i].c_str()))
            {
                std::vector< std::string > dp = vm[processKeys[i].c_str()].as< std::vector< std::string > >();

                if (dp.size() > 1)
                {
                    VR_WARNING << "More than one parameter for key " << processKeys[i] << ". taking the first one..." << endl;
                }

                if (dp.size() > 0)
                {
                    addKeyValuePair(processKeys[i], dp[0]);    // take the first one...
                }
            }

        }

        // collect unrecognized arguments
        std::vector<std::string> options = boost::program_options::collect_unrecognized(parsed.options, boost::program_options::include_positional);

        for (size_t i = 0; i < options.size(); i++)
        {
            unrecognizedOptions.push_back(options[i]);
        }


    }

    void RuntimeEnvironment::addKeyValuePair(const std::string& key, const std::string& value)
    {
        keyValues[key] = value;
    }

    std::string RuntimeEnvironment::getValue(const std::string& key)
    {
        if (keyValues.find(key) != keyValues.end())
        {
            return keyValues[key];
        }

        return std::string("");
    }

    std::map< std::string, std::string > RuntimeEnvironment::getKeyValuePairs()
    {
        return keyValues;
    }



    std::vector< std::string > RuntimeEnvironment::getDataPaths()
    {
        if (!pathInitialized)
        {
            init();
        }

        return dataPaths;
    }

    bool RuntimeEnvironment::addDataPath(const std::string& path, bool quiet)
    {
        try
        {
            boost::filesystem::path p(path);

            if (boost::filesystem::is_directory(p) || boost::filesystem::is_symlink(p))
            {
                dataPaths.push_back(path);
                return true;
            }
        }
        catch (...)
        {
        }

        if (!quiet)
        {
            VR_ERROR << "Trying to add non-existing data path: " << path << endl;
        }

        return false;
    }

    void RuntimeEnvironment::print()
    {
        if (!pathInitialized)
        {
            init();
        }

        cout << " *********** Simox RuntimeEnvironment ************* " << endl;
        cout << "Data paths:"  << endl;

        for (size_t i = 0; i < dataPaths.size(); i++)
        {
            cout << " * " << dataPaths[i] << endl;
        }

        if (processKeys.size() > 0)
        {
            cout << "Known keys:" << endl;

            for (size_t i = 0; i < processKeys.size(); i++)
            {
                cout << " * " << processKeys[i] << endl;
            }
        }

        if (keyValues.size() > 0)
        {
            cout << "Parsed options:"  << endl;
            std::map< std::string, std::string >::iterator it = keyValues.begin();

            while (it != keyValues.end())
            {
                cout << " * " << it->first << ": " << it->second << endl;
                it++;
            }
        }

        if (unrecognizedOptions.size() > 0)
        {
            cout << "Unrecognized options:" << endl;

            for (size_t i = 0; i < unrecognizedOptions.size(); i++)
            {
                cout << " * <" << unrecognizedOptions[i] << ">" << endl;
            }
        }

    }

    void RuntimeEnvironment::considerKey(const std::string& key)
    {
        processKeys.push_back(key);
    }

    bool RuntimeEnvironment::hasValue(const std::string& key)
    {
        return (keyValues.find(key) != keyValues.end());
    }

    bool RuntimeEnvironment::toVector3f(const std::string& s, Eigen::Vector3f& storeResult)
    {
        if (s.length() < 3)
        {
            return false;
        }

        if (s[0] != '(' || s[s.length() - 1] != ')')
        {
            VR_WARNING << "Expecting string to start and end with brackets (): " << s << endl;
            return false;
        }

        std::string s2 = s;
        s2.erase(s2.begin(), s2.begin() + 1);
        s2.erase(s2.end() - 1, s2.end());
        std::vector<std::string> strs;
        std::string del(",");

        boost::split(strs, s2, boost::is_any_of(del));

        if (strs.size() != 3)
        {
            VR_WARNING << "Expecting values of string to be separated with a ',': " << s << endl;
            return false;
        }

        float a = (float)atof(strs[0].c_str());
        float b = (float)atof(strs[1].c_str());
        float c = (float)atof(strs[2].c_str());

        if (boost::math::isinf(a) || boost::math::isinf(-a) || boost::math::isnan(a))
        {
            VR_WARNING << "Could not convert " << strs[0] << " to a number" << endl;
            return false;
        }

        if (boost::math::isinf(b) || boost::math::isinf(-b) || boost::math::isnan(b))
        {
            VR_WARNING << "Could not convert " << strs[1] << " to a number" << endl;
            return false;
        }

        if (boost::math::isinf(c) || boost::math::isinf(-c) || boost::math::isnan(c))
        {
            VR_WARNING << "Could not convert " << strs[2] << " to a number" << endl;
            return false;
        }

        storeResult(0) = a;
        storeResult(1) = b;
        storeResult(2) = c;
        return true;
    }

    std::string RuntimeEnvironment::checkValidFileParameter(const std::string& key, const std::string& standardFilename)
    {
        if (VirtualRobot::RuntimeEnvironment::hasValue(key))
        {
            std::string f = RuntimeEnvironment::getValue(key);

            if (VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(f))
            {
                return f;
            }
        }

        // don't check for empty files
        if (standardFilename.empty())
        {
            return standardFilename;
        }

        std::string s = standardFilename;

        if (!RuntimeEnvironment::getDataFileAbsolute(s))
        {
            VR_WARNING << "Could not determine path to file " << standardFilename << endl;
            return standardFilename;
        }

        return s;
    }

    std::string RuntimeEnvironment::checkParameter(const std::string& key, const std::string& standardValue /*= ""*/)
    {
        if (RuntimeEnvironment::hasValue(key))
        {
            return RuntimeEnvironment::getValue(key);
        }

        return standardValue;
    }

    void RuntimeEnvironment::cleanup()
    {
        VisualizationFactoryPtr visualizationFactory = VisualizationFactory::first(NULL);

        if (visualizationFactory)
        {
            visualizationFactory->cleanup();
        }
    }




} //  namespace


