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
#ifndef _Saba_MotionPlanner_h
#define _Saba_MotionPlanner_h

#include "../Saba.h"
#include "../CSpace/CSpace.h"
#include "../CSpace/CSpaceNode.h"
#include "../CSpace/CSpaceTree.h"

namespace Saba
{
    /*!
     *
     * \brief An abstract base class of a motion planner.
     *
     */
    class SABA_IMPORT_EXPORT MotionPlanner
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /*!
            constructor
        */
        MotionPlanner(CSpacePtr cspace);

        //! destructor
        virtual ~MotionPlanner();

        /*!
            do the planning (blocking method)
            \return true if solution was found, otherwise false
        */
        virtual bool plan(bool bQuiet = false) = 0;

        /*!
            Returns the solution path.
        */
        CSpacePathPtr getSolution();

        /*!
            Set maximal cycles. Initially set to 50000.
        */
        void setMaxCycles(unsigned int mc);

        //! return start configuration
        Eigen::VectorXf getStartConfig()
        {
            return startConfig;
        }

        //! return goal configuration
        Eigen::VectorXf getGoalConfig()
        {
            return goalConfig;
        }

        //! reset the planner
        virtual void reset();

        //! set start configuration
        virtual bool setStart(const Eigen::VectorXf& c);

        //! set goal configuration
        virtual bool setGoal(const Eigen::VectorXf& c);

        //! check that planner is initialized
        //virtual bool isInitialized();

        /*!
            Return number of cycles that were needed for motion planning
        */
        unsigned int getNrOfCycles()
        {
            return cycles;
        }

        /*!
            Print setup of planner.
            \param printOnlyParams If set the decorating start and end is skipped (can be used to print derived classes).
        */
        virtual void printConfig(bool printOnlyParams = false);

        //! The CSpace
        CSpacePtr getCSpace()
        {
            return cspace;
        }

        /*!
            Sets stop flag, so that this planner can be notified to abort the search.
            Only useful for threaded planners.
        */
        virtual void stopExecution()
        {
            stopSearch = true;
        }

        //! Give the planner a name
        void setName(std::string sName);

        //! The name of the planner.
        std::string getName();

        /*!
            Returns the time needed for planning (in milliseconds).
        */
        float getPlanningTimeMS()
        {
            return planningTime;
        }

        //! returns true, when start and goal config have been set
        virtual bool isInitialized();

    protected:

        //! create the solution
        virtual bool createSolution(bool bQuiet = false) = 0;
        CSpacePtr cspace;                   //!< the cspace on which are operating
        CSpacePathPtr solution;             //!< the solution

        bool stopSearch;                    //!< indicates that the search should be interrupted

        unsigned int dimension;             //!< dimension of c-space

        Eigen::VectorXf startConfig;        //!< start config
        bool startValid;
        Eigen::VectorXf goalConfig;         //!< goal config
        bool goalValid;

        unsigned int maxCycles;             //!< maximum cycles for searching
        unsigned int cycles;                //!< current cycles done in the run method

        std::string name;                   //!< Name of this planner (standard: "Motion Planner")

        float planningTime;                 //! Planning time in milliseconds
    };
}

#endif // _Saba_MotionPlanner_
