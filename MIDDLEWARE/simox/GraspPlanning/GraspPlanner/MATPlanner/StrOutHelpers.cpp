/**
* This file is part of Simox.
*
* Simox is free software; you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation; either version 2 of
* the License, or (at your option) any later version.
*
* Simox is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* @package    GraspStudio
* @author     Markus Przybylski
* @copyright  2013 H2T,KIT
*             GNU Lesser General Public License
*
*/
#include "StrOutHelpers.h"

using namespace std;

namespace GraspStudio
{

    StrOutHelpers::StrOutHelpers()
    {
    }

    string StrOutHelpers::toString(vector<Eigen::Vector3f> v, bool lineBreak)
    {
        stringstream sstr;
        sstr << endl;

        string endLine("");

        if (lineBreak)
        {
            endLine = "\n";
        }

        for (size_t i = 0; i < v.size(); i++)
        {
            sstr << "[" << (v.at(i))(0) << " " << (v.at(i))(1) << " " << (v.at(i))(2)
                 << "]; " << endLine;
        }

        return sstr.str();
    }

    string StrOutHelpers::toString(vector<Eigen::Vector3i> v, bool lineBreak)
    {
        stringstream sstr;
        sstr << endl;

        string endLine("");

        if (lineBreak)
        {
            endLine = "\n";
        }

        for (size_t i = 0; i < v.size(); i++)
        {
            sstr << "[" << (v.at(i))(0) << " " << (v.at(i))(1) << " " << (v.at(i))(2)
                 << "]; " << endLine;
        }

        return sstr.str();
    }

    string StrOutHelpers::toString(Eigen::Vector3f v)
    {
        stringstream sstr;
        sstr << endl;
        sstr << "[" << v(0) << " " << v(1) << " " << v(2) << "]; ";

        return sstr.str();
    }

    string StrOutHelpers::toString(Eigen::Vector3i v)
    {
        stringstream sstr;
        sstr << endl;
        sstr << "[" << v(0) << " " << v(1) << " " << v(2) << "]; ";

        return sstr.str();
    }
}
