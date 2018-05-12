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
* @package    Saba
* @author     Nikolaus Vahrenkamp
* @copyright  2012 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _Saba_ConfigConstraint_h
#define _Saba_ConfigConstraint_h

#include "../Saba.h"
#include "../CSpace/CSpace.h"
#include <vector>

namespace Saba
{

    /*!
    *
    * \brief An interface class for defining custom constraints
    *
    */
    class SABA_IMPORT_EXPORT ConfigurationConstraint
    {
    public:
        ConfigurationConstraint(unsigned int dimension);
        virtual ~ConfigurationConstraint();

        /*!
            An derived class has to implement this method in order to check
            if a configuration c satisfies the constraint or not.
            \param c The config to be tested.
            \return True if c satisfies the constraint.
        */
        virtual bool isValid(const Eigen::VectorXf& c) = 0;


    protected:

        unsigned int dimension;
    };

}

#endif // _Saba_ConfigConstraint_h
