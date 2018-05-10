#include "CustomLogger.h"
#include <ctime>

using namespace std;

#ifdef WIN32
#pragma warning(push)
#pragma warning(disable: 4996) //4996 for _CRT_SECURE_NO_WARNINGS equivalent
#endif

CustomLogger::CustomLogger(LogType lt, std::string filename)
{
    stdSep = ",";
    logtype = eConsole;

    if (lt == eFile && !filename.empty())
    {
        logfile.open(filename.c_str());

        if (logfile.is_open())
        {
            cout << "Logging to file " << filename.c_str() << endl;
            logtype = eFile;
        }
        else
        {
            cout << "Failed to open file " << filename.c_str() << " for logging" << endl;
            logtype = eConsole;
        }
    }
}

CustomLogger::~CustomLogger()
{
    close();
}

void CustomLogger::logString(const std::string& s, const std::string& sep, bool endline)
{
    ostream& o = getLogTarget();
    o << s.c_str();
    logTool(o, sep, endline);
}

ostream& CustomLogger::getLogTarget()
{
    switch (logtype)
    {
        case eFile:
            return logfile;
            break;

        default:
            return cout;
            break;
    }

    return cout;
}

void CustomLogger::logFloat(const float& f, const std::string& sep, bool endline)
{
    ostream& o = getLogTarget();
    o << f;
    logTool(o, sep, endline);
}

void CustomLogger::logInt(const int& f, const std::string& sep, bool endline)
{
    ostream& o = getLogTarget();
    o << f;
    logTool(o, sep, endline);
}

void CustomLogger::logLong(const long& f, const std::string& sep, bool endline)
{
    ostream& o = getLogTarget();
    o << f;
    logTool(o, sep, endline);
}

void CustomLogger::logDouble(const double& f, const std::string& sep, bool endline)
{
    ostream& o = getLogTarget();
    o << f;
    logTool(o, sep, endline);
}

void CustomLogger::logNewLine()
{
    ostream& o = getLogTarget();
    logTool(o, "", true);
}

void CustomLogger::logTool(ostream& o, const std::string& sep, bool endline)
{
    if (!sep.empty())
    {
        o << " " << sep.c_str() << " ";
    }

    if (endline)
    {
        o << endl;

        if (!preLogMessage.empty())
        {
            o << preLogMessage.c_str();
        }
    }
}

void CustomLogger::logFloatVector(vector<float> f, const std::string& sep, bool endline)
{
    ostream& o = getLogTarget();

    for (size_t i = 0; i < f.size(); i++)
    {
        o << f[i];
        logTool(o, sep, false);
    }

    logTool(o, "", endline);
}

void CustomLogger::close()
{
    if (logfile.is_open())
    {
        logfile.close();
        logtype = eConsole;
    }
}

void CustomLogger::setLogPreMessage(const std::string& m)
{
    preLogMessage = m;
}

void CustomLogger::logTimeStamp(bool endline /*= true*/)
{
    ostream& o = getLogTarget();

    // current date/time based on current system
    time_t now = time(0);

    // convert now to string form
    char* dt = ctime(&now);
    o << dt;

    logTool(o, "", endline);
}

void CustomLogger::log(double f)
{
    logDouble(f, stdSep, false);
}

void CustomLogger::log(int f)
{
    logInt(f, stdSep, false);
}

void CustomLogger::log(float f)
{
    logFloat(f, stdSep, false);
}

void CustomLogger::log(long f)
{
    logLong(f, stdSep, false);
}

void CustomLogger::log(const std::string& s)
{
    logString(s, stdSep, false);
}

void CustomLogger::logNewLine(double f)
{
    logDouble(f, "", true);
}

void CustomLogger::logNewLine(int f)
{
    logInt(f, "", true);
}

void CustomLogger::logNewLine(float f)
{
    logFloat(f, "", true);
}

void CustomLogger::logNewLine(long f)
{
    logLong(f, "", true);
}

void CustomLogger::logNewLine(const std::string& s)
{
    logString(s, "", true);
}
#ifdef WIN32
#pragma warning(pop)
#endif

