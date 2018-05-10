#ifndef __CUSTOM_LOGGER__
#define __CUSTOM_LOGGER__

#include <iostream>
#include <ostream>
#include <fstream>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#endif

class CustomLogger
{
public:
    enum LogType
    {
        eConsole, // standard
        eFile
    };

    CustomLogger(LogType lt = eConsole, std::string filename = "");

    ~CustomLogger();

    /*!
        Every new line starts with this message (disabled by default)
        \param m Set to empty string to disable
    */
    void setLogPreMessage(const std::string& m);


    void logString(const std::string& s, const std::string& sep = "", bool endline = true);
    void logInt(const int& f, const std::string& sep = "", bool endline = true);
    void logLong(const long& f, const std::string& sep = "", bool endline = true);
    void logDouble(const double& f, const std::string& sep = "", bool endline = true);
    void logFloat(const float& f, const std::string& sep = "", bool endline = true);
    void logFloatVector(std::vector<float> f, const std::string& sep = ",", bool endline = true);

    /*!
        Log without linebreak, the standard separator is added to log text (usually this is ',')
    */
    void log(double f);
    void log(int f);
    void log(float f);
    void log(long f);
    void log(const std::string& s);
    void logNewLine(double f);
    void logNewLine(int f);
    void logNewLine(float f);
    void logNewLine(long f);
    void logNewLine(const std::string& s);

    void logTimeStamp(bool endline = true);


    void setLogSeparator(const std::string& s);

    void logNewLine();
    /*!
        Only useful when in eFile mode.
        Logging after closing is redirected to the console.
    */
    void close();

protected:
    void logTool(std::ostream& o, const std::string& sep, bool endline);
    std::ostream& getLogTarget();
    std::ofstream logfile;
    LogType logtype;

    std::string preLogMessage;
    std::string stdSep;

};

typedef boost::shared_ptr<CustomLogger> CustomLoggerPtr;

#endif // __CUSTOM_LOGGER__
