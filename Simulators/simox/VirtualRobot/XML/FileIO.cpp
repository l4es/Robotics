
#include "FileIO.h"
#include "../VirtualRobotException.h"

std::vector< Eigen::Vector3f > VirtualRobot::FileIO::readPts(const std::string& filename, const char separator)
{
    std::ifstream file(filename.c_str());
    THROW_VR_EXCEPTION_IF(!file.good(), "Could not open file" << filename);
    char tmp;
    float a, b, c;
    std::vector< Eigen::Vector3f > res;
    Eigen::Vector3f v;
    bool needToReadSep = true;

    if (separator == ' ' || separator == '\n')
    {
        needToReadSep = false;
    }

    while (file.good())
    {
        file >> a;

        //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        if (needToReadSep)
        {
            file >> tmp;
            //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        }

        file >> b;

        //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        if (needToReadSep)
        {
            file >> tmp;
            //THROW_VR_EXCEPTION_IF(!file.good(),"Error in file " << filename << " line " << res.size());
        }

        file >> c;
        v << a, b, c;
        res.push_back(v);
    }

    return res;
}

