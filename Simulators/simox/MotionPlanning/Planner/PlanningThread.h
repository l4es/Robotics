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
#ifndef _Saba_PlanningThread_h
#define _Saba_PlanningThread_h

#include "../Saba.h"
#include "../CSpace/CSpaceSampled.h"
#include "../CSpace/CSpacePath.h"
#include <VirtualRobot/VirtualRobot.h>
#include "MotionPlanner.h"


namespace Saba
{

    /*!
     *
     * The GraspIkRrt planner combines the search for a feasible grasp and an IK solution with the search for a collision-free motion.
     *
     */
    class SABA_IMPORT_EXPORT PlanningThread
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        /*! Constructor
            The thread is not started until you call start()
            \param planner The planner to be started.
        */
        PlanningThread(MotionPlannerPtr planner);

        //! destructor
        virtual ~PlanningThread();

        /*!
            Start the planning in an own thread.
        */
        virtual void start();

        /*!
            Send an interrupt signal to thread.
            \param waitUntilStopped If false this method returns immediately. Otherwise we wait until the thread has been successfully interrupted.
        */
        virtual void interrupt(bool waitUntilStopped = false);

        /*!
            Same as interrupt(true)
        */
        void stop();

        /*!
            \return True if the planning thread is operating
        */
        bool isRunning();

        MotionPlannerPtr getPlanner();

    protected:

        /*!
            Here the planning takes place.
        */
        void workingMethod();


        bool threadStarted;
        bool plannerFinished;
        MotionPlannerPtr planner;
        boost::thread planningThread;
        boost::mutex mutex;

    };

}
#endif // _Saba_PlanningThread_h
