#ifndef _VirtualRobot_Units_h_
#define _VirtualRobot_Units_h_

/**
* @package    VirtualRobot
* @author     Manfred Kroehnert <mkroehnert _at_ users dot sourceforge dot net>
* @copyright  2011 Manfred Kroehnert
*/

#include "VirtualRobotImportExport.h"
#include <string>
#include <algorithm>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT Units
    {
    public:

        enum UnitsType
        {
            eAngle,
            eLength,
            eWeight,
            eTime,
            eIgnore
        };

        Units(const std::string& unitName) : unitString(unitName)
        {
            //std::transform(unitString.begin(), unitString.end(), unitString.begin(), ::tolower);
        }

        bool isRadian()
        {
            return ("radian" == unitString || "rad" == unitString);
        }
        bool isDegree()
        {
            return ("degree" == unitString || "deg" == unitString);
        }
        bool isAngle()
        {
            return (isRadian() ||  isDegree());
        }
        float toRadian(float m)
        {
            if (isDegree())
            {
                return m * (float)M_PI / 180.0f;
            }
            else
            {
                return m;
            }
        }
        float toDegree(float m)
        {
            if (isRadian())
            {
                return m * 180.0f / (float)M_PI;
            }
            else
            {
                return m;
            }
        }

        bool isMillimeter()
        {
            return ("mm" == unitString || "millimeter" == unitString);
        }
        bool isMeter()
        {
            return ("m" == unitString || "meter" == unitString);
        }
        bool isLength()
        {
            return (isMillimeter() ||  isMeter());
        }
        float toMillimeter(float m)
        {
            if (isMeter())
            {
                return m * 1000.0f;
            }
            else
            {
                return m;
            }
        }
        float toMeter(float m)
        {
            if (isMillimeter())
            {
                return m * 0.001f;
            }
            else
            {
                return m;
            }
        }

        bool isGram()
        {
            return ("g" == unitString || "gram" == unitString);
        }
        bool isKilogram()
        {
            return ("kg" == unitString || "kilogram" == unitString);
        }
        bool isTon()
        {
            return ("t" == unitString || "ton" == unitString);
        }
        bool isWeight()
        {
            return (isGram() ||  isKilogram() ||  isTon());
        }
        float toGram(float m)
        {
            if (isKilogram())
            {
                return m * 1000.0f;
            }
            else if (isTon())
            {
                return m * 1000000.0f;
            }
            else
            {
                return m;
            }
        }
        float toKilogram(float m)
        {
            if (isGram())
            {
                return m * 0.001f;
            }
            else if (isTon())
            {
                return m * 1000.0f;
            }
            else
            {
                return m;
            }
        }
        float toTon(float m)
        {
            if (isGram())
            {
                return m * 0.000001f;
            }
            else if (isKilogram())
            {
                return m * 0.001f;
            }
            else
            {
                return m;
            }
        }

        bool isSecond()
        {
            return ("s" == unitString || "sec" == unitString || "second" == unitString);
        }
        bool isMinute()
        {
            return ("min" == unitString || "minute" == unitString);   //!< be careful m==meter!
        }
        bool isHour()
        {
            return ("h" == unitString || "hour" == unitString);
        }
        bool isTime()
        {
            return (isSecond() ||  isMinute() ||  isHour());
        }
        float toSecond(float m)
        {
            if (isMinute())
            {
                return m * 60.0f;
            }
            else if (isHour())
            {
                return m * 3600.0f;
            }
            else
            {
                return m;
            }
        }
        float toMinute(float m)
        {
            if (isSecond())
            {
                return m / 60.0f;
            }
            else if (isHour())
            {
                return m * 60.0f;
            }
            else
            {
                return m;
            }
        }
        float toHour(float m)
        {
            if (isSecond())
            {
                return m / 3600.0f;
            }
            else if (isMinute())
            {
                return m / 60.0f;
            }
            else
            {
                return m;
            }
        }

        bool isValid()
        {
            return (isLength() || isAngle() || isWeight() || isTime());
        }

    private:
        std::string unitString;
    };

} // namespace VirtualRobot

#endif /* _VirtualRobot_Units_h_ */
