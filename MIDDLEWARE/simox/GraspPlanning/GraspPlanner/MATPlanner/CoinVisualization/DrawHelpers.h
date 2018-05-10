#ifndef DRAWHELPERS_H
#define DRAWHELPERS_H


#include <Eigen/Geometry>

#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoCoordinate3.h>

#include <vector>
#include "../../../GraspStudio.h"
#include "../MedialSphere.h"
#include "../LocalNeighborhood.h"
#include "../StrOutHelpers.h"
#include "../CandidateGrasp.h"

namespace GraspStudio
{
    class GRASPSTUDIO_IMPORT_EXPORT DrawHelpers
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        DrawHelpers();

        static SoSeparator* DrawSingleSphere(Eigen::Vector3f& position, float radius, float colorR, float colorG, float colorB, float transparency);

        static SoSeparator* DrawSingleSphere(MedialSpherePtr ms, float colorR, float colorG, float colorB, float transparency);

        static SoSeparator* DrawSinglePoint(MedialSpherePtr ms, float colorR, float colorG, float colorB, float transparency, float pointSize);

        static SoSeparator* DrawSinglePoint(Eigen::Vector3f& position, float colorR, float colorG, float colorB, float transparency, float pointSize);

        static SoSeparator* DrawVectorOfSpheres(std::vector<MedialSpherePtr>& spheres, float maxRadius, float transparency);

        static SoSeparator* DrawPointCloud(std::vector<MedialSpherePtr>& spheres, float colorR, float colorG, float colorB, float transparency, float pointSize);

        static SoSeparator* DrawPointCloud(std::vector<Eigen::Vector3f>& points, float colorR, float colorG, float colorB, float transparency, float pointSize);

        static SoSeparator* DrawSingleLine(Eigen::Vector3f from, Eigen::Vector3f to, float width, float colorR, float colorG, float colorB);

        static SoSeparator* DrawLocalNeighborhood(LocalNeighborhoodPtr neighborhood,
                bool drawEigenvectors, bool drawCenterOfGravity, bool drawSearchRadius,
                float pointSize, float scaleValue);

        static SoSeparator* DrawLocalNeighborhoods(std::vector<LocalNeighborhoodPtr>& neighborhoods,
                bool drawEigenvectors, bool drawCenterOfGravity, bool drawSearchRadius,
                float pointSize, float scaleValue);

        static SoSeparator* DrawSearchRadius(LocalNeighborhoodPtr neighborhood);
        static SoSeparator* DrawSearchRadii(std::vector<LocalNeighborhoodPtr> neighborhoods);

        static SoSeparator* DrawCandidateGrasp(CandidateGraspPtr cg, float scaleValue);

        static SoSeparator* DrawCandidateGrasps(std::vector<CandidateGraspPtr>& cg, float scaleValue);

        static Eigen::Vector3f getColorFromTable(float currentValue, float kFactor, float maxInputValue);
    };

}

#endif // DRAWHELPERS_H
