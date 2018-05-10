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
#ifndef __Saba_PathProcessingThread_h__
#define __Saba_PathProcessingThread_h__

#include "../Saba.h"

#include "PathProcessor.h"
#include "../CSpace/CSpacePath.h"


namespace Saba
{
    /*!
     *
     * \brief This class can be used to start a path processing algorithm in a thread.
     *
     */
    class SABA_IMPORT_EXPORT PathProcessingThread
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


        /*! Constructor
            The thread is not started until you call start()
            \param processor An initialized path processor to be started.
        */
        PathProcessingThread(PathProcessorPtr processor);

        //! destructor
        virtual ~PathProcessingThread();

        /*!
            Start the path processing in an own thread.
        */
        virtual void start(int optimizeSteps);

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

        PathProcessorPtr getPathProcessor();

        //! Returns the optimized path (when optimizer is finished)
        CSpacePathPtr getProcessedPath();

    protected:

        /*!
            Here the post processing takes place.
        */
        void workingMethod();


        bool threadStarted;
        bool processingFinished;
        PathProcessorPtr pathProcessor;
        boost::thread processingThread;
        boost::mutex mutex;

        CSpacePathPtr resultPath;
        int optimizeSteps;

    };

}

#endif // _PostprocessingThread_h
