
#include "ConvexHullVisualization.h"


namespace GraspStudio
{

    ConvexHullVisualization::ConvexHullVisualization(VirtualRobot::MathTools::ConvexHull6DPtr convHull, bool useFirst3Coords)
    {
        this->convHull6D = convHull;
        this->useFirst3Coords = useFirst3Coords;

    }

    ConvexHullVisualization::ConvexHullVisualization(VirtualRobot::MathTools::ConvexHull3DPtr convHull)
    {
        this->convHull3D = convHull;
        this->useFirst3Coords = true;
    }


    ConvexHullVisualization::~ConvexHullVisualization()
    {
    }

} // namespace GraspStudio
