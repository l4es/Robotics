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
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef __Saba_PathProcessor_h__
#define __Saba_PathProcessor_h__

#include "../Saba.h"

namespace Saba
{
    /*!
     *
     * \brief An abstract interface for path processing classes.
     *
     */
    class SABA_IMPORT_EXPORT PathProcessor
    {
    public:

        /*!
            Constructor
            Creates a local copy of p.
        */
        PathProcessor(CSpacePathPtr p, bool verbose = false);

        /*!
            Destructor
            Deletes local optimized path.
        */
        virtual ~PathProcessor();


        /*!
            Here the path processing is executed, the number of optimizing steps can be specified.
        */
        virtual CSpacePathPtr optimize(int optimizeSteps) = 0;

        //! Stop the execution from outside.
        virtual void stopExecution();

        CSpacePathPtr getOptimizedPath();

    protected:

        CSpacePathPtr optimizedPath;        // the optimized path
        CSpacePathPtr path;                 // stores the original path
        unsigned int dim;
        bool verbose;

        bool stopOptimization; // allows to stop a running optimization process
    };

} // namespace

#endif // __Saba_CPathProcessor_h__
