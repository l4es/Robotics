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
#ifndef _Saba_BiRrt_h
#define _Saba_BiRrt_h

#include "../Saba.h"
#include "../CSpace/CSpaceSampled.h"
#include "../CSpace/CSpacePath.h"
#include "Rrt.h"

namespace Saba
{


    /*!
     * The standard implementation of the bidirectional RRT planner.
     * Two search trees are started, one from the start and one from the goal node.
     *
     */
    class SABA_IMPORT_EXPORT BiRrt : public Rrt
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            Constructor
            \param cspace An initialized cspace object.
            \param modeA Specify the RRT method that should be used to build the first tree
            \param modeB Specify the RRT method that should be used to build the second tree
        */
        BiRrt(CSpaceSampledPtr cspace, RrtMethod modeA = eConnect, RrtMethod modeB = eConnect);
        virtual ~BiRrt();

        /*!
            do the planning (blocking method)
            \return true if solution was found, otherwise false
        */
        virtual bool plan(bool bQuiet = false);


        virtual void printConfig(bool printOnlyParams = false);

        virtual void reset();

        //! set start configuration
        virtual bool setStart(const Eigen::VectorXf& c);

        //! set goal configuration
        virtual bool setGoal(const Eigen::VectorXf& c);

        CSpaceTreePtr getTree2();

    protected:

        virtual bool createSolution(bool bQuiet = false);

        CSpaceTreePtr tree2;                    //!< the second tree

        int lastAddedID2;               //!< ID of last added node
        RrtMethod rrtMode2;
    };

} // namespace

#endif // _Saba_RRT_h


