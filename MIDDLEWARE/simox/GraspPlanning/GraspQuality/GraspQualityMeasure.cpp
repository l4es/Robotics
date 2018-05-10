// **************************************************************
// Implementation of class GraspQualityMeasure
// **************************************************************
// Author: Niko Vahrenkamp
// Date: 26.10.2011
// **************************************************************


// **************************************************************
// includes
// **************************************************************

#include "GraspQualityMeasure.h"
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/CollisionDetection/CollisionModel.h>
#include <VirtualRobot/Visualization/TriMeshModel.h>

using namespace std;
using namespace VirtualRobot;

namespace GraspStudio
{


    GraspQualityMeasure::GraspQualityMeasure(VirtualRobot::SceneObjectPtr object, float unitForce, float frictionConeCoeff, int frictionConeSamples)
        : VirtualRobot::BasicGraspQualityMeasure(object), unitForce(unitForce), frictionCoeff(frictionConeCoeff), frictionConeSamples(frictionConeSamples)
    {
        coneGenerator.reset(new ContactConeGenerator(frictionConeSamples, frictionCoeff, unitForce));
    }

    GraspQualityMeasure::~GraspQualityMeasure()
    {
    }

    bool GraspQualityMeasure::sampleObjectPoints(int nMaxFaces)
    {
        sampledObjectPoints.clear();
        sampledObjectPointsM.clear();

        TriMeshModelPtr model = object->getCollisionModel()->getTriMeshModel();
        //Eigen::Vector3f _com = model->getCOM();
        MathTools::ContactPoint objectPoint;

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": object COM: << " << centerOfModel[0] << "," << centerOfModel[1] << "," << centerOfModel[2] << endl;
        }

        int nFaces = (int)model->faces.size();

        if (nFaces > nMaxFaces)
        {
            nFaces = nMaxFaces;
        }

        int nLoopCount = 0;
        std::vector<MathTools::TriangleFace> vFaceCopy = model->faces;
        std::vector<MathTools::TriangleFace>::iterator iFaceIter;

        while (nLoopCount < nFaces && vFaceCopy.size() > 0)
        {
            int nRnd = rand() % vFaceCopy.size();
            iFaceIter = vFaceCopy.begin();
            iFaceIter += nRnd;

            objectPoint.p = (model->vertices[iFaceIter->id1] + model->vertices[iFaceIter->id2] + model->vertices[iFaceIter->id3]) / 3.0f;
            objectPoint.n = iFaceIter->normal;
            objectPoint.n.normalize();
            objectPoint.n *= unitForce;
            objectPoint.force = 1.0f;

            // move points so that object is located at origin
            objectPoint.p -= centerOfModel;
            sampledObjectPoints.push_back(objectPoint);

            vFaceCopy.erase(iFaceIter);
            nLoopCount++;
        }

        MathTools::convertMM2M(sampledObjectPoints, sampledObjectPointsM);

        if (verbose)
        {
            GRASPSTUDIO_INFO << ": Nr of sample object points:" << sampledObjectPoints.size() << endl;
        }

        return (sampledObjectPoints.size() > 0);
    }


    MathTools::ContactPoint GraspQualityMeasure::getSampledObjectPointsCenter()
    {
        MathTools::ContactPoint p;
        p.p.setZero();
        p.n.setZero();

        if (sampledObjectPoints.size() == 0)
        {
            return p;
        }

        for (int i = 0; i < (int)sampledObjectPoints.size(); i++)
        {
            p.p += sampledObjectPoints[i].p;
            p.n += sampledObjectPoints[i].n;
        }

        p.p /= (float)sampledObjectPoints.size();
        p.n /= (float)sampledObjectPoints.size();
        return p;
    }

    std::string GraspQualityMeasure::getName()
    {
        std::string sName("GraspQualityMeasure");
        return sName;
    }

    bool GraspQualityMeasure::isValid()
    {
        return isGraspForceClosure();
    }

    GraspStudio::ContactConeGeneratorPtr GraspQualityMeasure::getConeGenerator()
    {
        return coneGenerator;
    }

} // namespace
