
#include "PathProcessor.h"
#include "../CSpace/CSpaceSampled.h"
#include "../CSpace/CSpacePath.h"

namespace Saba
{
    PathProcessor::PathProcessor(CSpacePathPtr path, bool verbose)
        : path(path), verbose(verbose)
    {
        THROW_VR_EXCEPTION_IF((!path), "NULl path...");
        stopOptimization = false;
    }

    PathProcessor::~PathProcessor()
    {
    }

    CSpacePathPtr PathProcessor::getOptimizedPath()
    {
        return optimizedPath;
    }

    void PathProcessor::stopExecution()
    {
        stopOptimization = true;
    }
}
