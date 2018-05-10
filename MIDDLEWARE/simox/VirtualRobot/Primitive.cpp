#include "Primitive.h"
#include "MathTools.h"


namespace VirtualRobot
{
    namespace Primitive
    {

        std::string Primitive::getTransformString(int tabs)
        {
            std::stringstream result;
            std::string pre;

            for (int i = 0; i < tabs; i++)
            {
                pre += "\t";
            }

            result << pre << "\t<Transform>\n";
            result << MathTools::getTransformXMLString(transform, tabs + 2);
            result << pre << "\t</Transform>\n";
            return result.str();
        }

        std::string Primitive::getXMLString(const std::string& type, const std::string& params, int tabs)
        {
            std::stringstream result;
            std::string pre;

            for (int i = 0; i < tabs; i++)
            {
                pre += "\t";
            }

            result << pre << "\t<" << type << " " << params << ">\n";
            result << getTransformString(tabs + 1);
            result << pre << "\t</" << type << ">\n";
            return result.str();
        }

        //derivate functions
        std::string Box::toXMLString(int tabs)
        {
            return getXMLString(
                       "Box",
                       (boost::format("width=\"%f\" height=\"%f\" depth=\"%f\"") % width % height % depth).str(),
                       tabs);
        }

        std::string Sphere::toXMLString(int tabs)
        {
            return getXMLString(
                       "Sphere",
                       (boost::format("radius=\"%f\"") % radius).str(),
                       tabs);
        }

        std::string Cylinder::toXMLString(int tabs)
        {
            return getXMLString(
                       "Cylinder",
                       (boost::format("radius=\"%f\" height=\"%f\"") % radius % height).str(),
                       tabs);
        }

    } //namespace Primitive
} //namespace VirtualRobot
