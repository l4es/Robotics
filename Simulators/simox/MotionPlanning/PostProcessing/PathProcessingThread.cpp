
#include "PathProcessingThread.h"


#include "PathProcessingThread.h"

#include <time.h>
#include <iostream>

using namespace std;

namespace Saba
{

    //! constructor
    PathProcessingThread::PathProcessingThread(PathProcessorPtr processor)
    {
        pathProcessor = processor;
        THROW_VR_EXCEPTION_IF(!processor, "NULL data");
        threadStarted = false;
        processingFinished = false;
    }

    //! destructor
    PathProcessingThread::~PathProcessingThread()
    {
        stop();
    }

    void PathProcessingThread::start(int optimizeSteps)
    {
        boost::lock_guard<boost::mutex> lock(mutex);

        if (threadStarted)
        {
            // thread already started, nothing to do
            return;
        }

        // since the start method is the only way to start the thread we can set the variables for indicating the state.
        threadStarted = true;
        processingFinished = false;
        this->optimizeSteps = optimizeSteps;
        resultPath.reset();

        processingThread = boost::thread(&PathProcessingThread::workingMethod, this);
    }

    void PathProcessingThread::interrupt(bool waitUntilStopped)
    {
        if (!isRunning())
        {
            // thread not started, nothing to do
            return;
        }

        // this is not perfect: we are setting a bool without protecting it with a mutex. But it works...
        if (pathProcessor)
        {
            pathProcessor->stopExecution();
        }

        // todo: catch boost::thread_interrupted in MotionPlanners and be sure to call boost::threa::interrupt points during planning...
        //thread.interrupt();

        if (waitUntilStopped)
        {
            processingThread.join();
        }
    }


    void PathProcessingThread::stop()
    {
        interrupt(true);
    }

    bool PathProcessingThread::isRunning()
    {
        boost::lock_guard<boost::mutex> lock(mutex);
        return threadStarted;
    }

    PathProcessorPtr PathProcessingThread::getPathProcessor()
    {
        return pathProcessor;
    }

    void PathProcessingThread::workingMethod()
    {
        if (!threadStarted)
        {
            VR_WARNING << "Thread should be in started mode?!" << endl;
        }

        VR_ASSERT(pathProcessor);

        CSpacePathPtr res = pathProcessor->optimize(optimizeSteps);

        mutex.lock();

        if (res)
        {
            processingFinished = true;
        }
        else
        {
            processingFinished = false;
        }

        resultPath = res;

        // the thread ends here
        threadStarted = false;
        mutex.unlock();
    }

    Saba::CSpacePathPtr PathProcessingThread::getProcessedPath()
    {
        if (isRunning())
        {
            return Saba::CSpacePathPtr();    // no results yet
        }

        return resultPath;
    }

}

