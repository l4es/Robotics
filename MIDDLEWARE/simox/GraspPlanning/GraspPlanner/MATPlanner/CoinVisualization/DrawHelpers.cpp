#include <iostream>

#include "DrawHelpers.h"

using namespace std;
using namespace Eigen;

namespace GraspStudio
{


    DrawHelpers::DrawHelpers()
    {
    }

    SoSeparator* DrawHelpers::DrawSingleSphere(Eigen::Vector3f& position, float radius, float colorR, float colorG, float colorB, float transparency)
    {
        SoSeparator* s = new SoSeparator();
        s->ref();

        SoTranslation* tr = new SoTranslation();
        tr->translation.setValue(position(0), position(1), position(2));
        s->addChild(tr);

        SoUnits* u = new SoUnits();
        //u->units = SoUnits::MILLIMETERS;
        u->units = SoUnits::METERS;
        s->addChild(u);

        SoMaterial* m = new SoMaterial();
        s->addChild(m);
        //m->ambientColor.setValue(colorR,colorG,colorB);
        m->ambientColor.setValue(0, 0, 0);
        m->diffuseColor.setValue(colorR, colorG, colorB);
        m->transparency.setValue(transparency);

        SoSphere* c = new SoSphere();
        s->addChild(c);
        c->radius = radius;

        return s;
    }

    SoSeparator* DrawHelpers::DrawSingleSphere(MedialSpherePtr ms, float colorR, float colorG, float colorB, float transparency)
    {
        SoSeparator* sep = DrawSingleSphere(ms->center, ms->radius, colorR, colorG, colorB, transparency);
        return sep;
    }

    SoSeparator* DrawHelpers::DrawSinglePoint(MedialSpherePtr ms, float colorR, float colorG, float colorB, float transparency, float pointSize)
    {
        SoSeparator* sep = DrawSingleSphere(ms->center, pointSize, colorR, colorG, colorB, transparency);
        return sep;
    }

    SoSeparator* DrawHelpers::DrawSinglePoint(Eigen::Vector3f& position, float colorR, float colorG, float colorB, float transparency, float pointSize)
    {
        SoSeparator* sep = DrawSingleSphere(position, pointSize, colorR, colorG, colorB, transparency);
        return sep;
    }

    SoSeparator* DrawHelpers::DrawPointCloud(std::vector<MedialSpherePtr>& spheres, float colorR, float colorG, float colorB, float transparency, float pointSize)
    {
        SoSeparator* sphereVectorSep = new SoSeparator();
        sphereVectorSep->ref();

        for (size_t i = 0; i < spheres.size(); i++)
        {
            SoSeparator* sphereSep = DrawSinglePoint(spheres.at(i), colorR, colorG, colorB, transparency, pointSize);
            sphereVectorSep->addChild(sphereSep);
        }

        return sphereVectorSep;
    }

    SoSeparator* DrawHelpers::DrawPointCloud(std::vector<Eigen::Vector3f>& points, float colorR, float colorG, float colorB, float transparency, float pointSize)
    {
        SoSeparator* sphereVectorSep = new SoSeparator();
        sphereVectorSep->ref();

        for (size_t i = 0; i < points.size(); i++)
        {
            SoSeparator* sphereSep = DrawSinglePoint(points.at(i), colorR, colorG, colorB, transparency, pointSize);
            sphereVectorSep->addChild(sphereSep);
        }

        return sphereVectorSep;
    }

    SoSeparator* DrawHelpers::DrawVectorOfSpheres(std::vector<MedialSpherePtr>& spheres, float maxRadius, float transparency)
    {
        float kFactorForColor = 1.1f;
        float maxValue = maxRadius; //biggest sphere of the MAT

        SoSeparator* sphereVectorSep = new SoSeparator();
        sphereVectorSep->ref();

        for (size_t i = 0; i < spheres.size(); i++)
        {
            MedialSpherePtr currentSphere = spheres.at(i);
            Vector3f currentColor = DrawHelpers::getColorFromTable(currentSphere->radius, kFactorForColor, maxValue);

            SoSeparator* sphereSep = DrawSingleSphere(currentSphere, currentColor(0), currentColor(1), currentColor(2), transparency);

            sphereVectorSep->addChild(sphereSep);
        }

        return sphereVectorSep;
    }

    SoSeparator* DrawHelpers::DrawSingleLine(Eigen::Vector3f from, Eigen::Vector3f to, float width, float colorR, float colorG, float colorB)
    {
        SoSeparator* s = new SoSeparator();
        s->ref();
        SoUnits* u = new SoUnits();
        //u->units = SoUnits::MILLIMETERS;
        u->units = SoUnits::METERS; //MP 2013-09-13 Test...
        s->addChild(u);

        SoMaterial* m = new SoMaterial();
        s->addChild(m);
        m->ambientColor.setValue(colorR, colorG, colorB);
        m->diffuseColor.setValue(colorR, colorG, colorB);

        // create line
        float x = from(0);
        float y = from(1);
        float z = from(2);
        float x2 = to(0);
        float y2 = to(1);
        float z2 = to(2);

        SbVec3f points[2];
        points[0].setValue(x2, y2, z2);
        points[1].setValue(x, y, z);

        SoDrawStyle* lineSolutionStyle = new SoDrawStyle();
        lineSolutionStyle->lineWidth.setValue(width);
        s->addChild(lineSolutionStyle);

        SoCoordinate3* coordinate3 = new SoCoordinate3;
        coordinate3->point.set1Value(0, points[0]);
        coordinate3->point.set1Value(1, points[1]);
        s->addChild(coordinate3);

        SoLineSet* lineSet = new SoLineSet;
        lineSet->numVertices.setValue(2);
        lineSet->startIndex.setValue(0);
        s->addChild(lineSet);
        s->unrefNoDelete();

        return s;
    }

    SoSeparator* DrawHelpers::DrawLocalNeighborhood(LocalNeighborhoodPtr neighborhood, bool drawEigenvectors, bool drawCenterOfGravity, bool drawSearchRadius, float pointSize, float scaleValue)
    {
        SoSeparator* neighborhoodSep = new SoSeparator();
        neighborhoodSep->ref();

        //1. Draw the center of the query sphere:
        float kFactorForColor = 1.1f;
        float maxValue = 1.0; //maximum possible value of ratioOfEigenvalues
        float pointTransparency = 0.0;
        Vector3f centerColor = DrawHelpers::getColorFromTable(neighborhood->ratioOfEigenvalues,
                               kFactorForColor, maxValue);
        SoSeparator* centerSep = DrawSinglePoint(neighborhood->center,
                                 centerColor[0], centerColor[1], centerColor[2],
                                 pointTransparency, pointSize);
        neighborhoodSep->addChild(centerSep);

        //2. Draw eigenvectors
        if (drawEigenvectors)
        {
            Eigen::Vector3f endPoint1, endPoint2;
            float lineWidth = 2.0;
            endPoint1 = neighborhood->center + scaleValue * neighborhood->eigenvector1;
            endPoint2 = neighborhood->center + scaleValue * neighborhood->eigenvector2;

            SoSeparator* sepLine1 = DrawHelpers::DrawSingleLine(neighborhood->center,
                                    endPoint1, lineWidth, 1.0, 0.0, 0.0);
            neighborhoodSep->addChild(sepLine1);
            SoSeparator* sepLine2 = DrawHelpers::DrawSingleLine(neighborhood->center,
                                    endPoint2, lineWidth, 0.0, 1.0, 0.0);
            neighborhoodSep->addChild(sepLine2);
        }

        //3. Draw center of gravity (cog)
        if (drawCenterOfGravity)
        {
            //center of gravity
            SoSeparator* cogSep = DrawSinglePoint(neighborhood->centerOfGravity,
                                                  0.0f, 0.0f, 1.0f,
                                                  pointTransparency, 0.5f * pointSize);
            neighborhoodSep->addChild(cogSep);

            //line from sphere center to cog
            float cogLineWidth = 1.0;
            SoSeparator* sepCogLine = DrawHelpers::DrawSingleLine(
                                          neighborhood->centerOfGravity, neighborhood->center,
                                          cogLineWidth, 0.0, 0.0, 1.0);
            neighborhoodSep->addChild(sepCogLine);
        }

        //4. Draw search radius (as a transparent sphere)
        if (drawSearchRadius)
        {
            float neighborhoodTransparency = 0.975f;
            SoSeparator* searchRadiusSep = DrawSingleSphere(neighborhood->center,
                                           neighborhood->radius, 0.0f, 0.2f, 0.4f, neighborhoodTransparency);
            neighborhoodSep->addChild(searchRadiusSep);
        }

        return neighborhoodSep;
    }

    SoSeparator* DrawHelpers::DrawCandidateGrasp(CandidateGraspPtr cg, float scaleValue)
    {
        Eigen::Vector3f startPoint, orientationPoint;
        float lineWidth = 1.0;
        startPoint = cg->graspTargetPoint + scaleValue * cg->handApproachDirection;
        orientationPoint = startPoint + 0.15f * scaleValue * cg->handOrientation;
        Eigen::Vector3f approachColor;
        Eigen::Vector3f orientationColor;

        if (cg->candidateGraspType.compare("symmetryAxisOrthogonal") == 0)
        {
            approachColor << 1.0f, 0.35f, 0.15f;   //orange
        }
        else
        {
            if (cg->candidateGraspType.compare("symmetryPlaneParallel") == 0)
            {
                approachColor << 0.0f, 0.7f, 0.3f; //green
            }
            else
            {
                approachColor << 0.0f, 0.0f, 0.0f; //black
            }
        }

        orientationColor << 1.0f, 0.0f, 0.5f; //magenta

        SoSeparator* cgSep = new SoSeparator();
        cgSep->ref();

        //approach direction
        SoSeparator* sepLine1 = DrawHelpers::DrawSingleLine(cg->graspTargetPoint,
                                startPoint, lineWidth, approachColor[0],
                                approachColor[1], approachColor[2]);
        cgSep->addChild(sepLine1);

        //hand orientation
        SoSeparator* sepLine2 = DrawHelpers::DrawSingleLine(startPoint,
                                orientationPoint, lineWidth,
                                orientationColor[0], orientationColor[1], orientationColor[2]);
        cgSep->addChild(sepLine2);

        return cgSep;
    }

    SoSeparator* DrawHelpers::DrawLocalNeighborhoods(std::vector<LocalNeighborhoodPtr>& neighborhoods, bool drawEigenvectors, bool drawCenterOfGravity, bool drawSearchRadius, float pointSize, float scaleValue)
    {
        SoSeparator* sepNeighborhoods = new SoSeparator;
        sepNeighborhoods->ref();

        for (int i = 0; i < (int)neighborhoods.size(); i++)
        {
            SoSeparator* sepCurrentNbhd =
                DrawHelpers::DrawLocalNeighborhood(neighborhoods.at(i),
                                                   drawEigenvectors, drawCenterOfGravity, drawSearchRadius,
                                                   pointSize, scaleValue);
            sepNeighborhoods->addChild(sepCurrentNbhd);
        }

        return sepNeighborhoods;
    }


    SoSeparator* DrawHelpers::DrawSearchRadius(LocalNeighborhoodPtr neighborhood)
    {
        float neighborhoodTransparency = 0.99f;
        SoSeparator* searchRadiusSep = DrawSingleSphere(neighborhood->center,
                                       neighborhood->radius, 0.0f, 0.2f, 0.4f,
                                       neighborhoodTransparency);
        return searchRadiusSep;
    }

    SoSeparator* DrawHelpers::DrawSearchRadii(std::vector<LocalNeighborhoodPtr> neighborhoods)
    {
        SoSeparator* sepRadii = new SoSeparator;
        sepRadii->ref();

        for (size_t i = 0; i < neighborhoods.size(); i++)
        {
            SoSeparator* sepCurrentNbhd =
                DrawHelpers::DrawSearchRadius(neighborhoods.at(i));
            sepRadii->addChild(sepCurrentNbhd);
        }

        return sepRadii;
    }

    SoSeparator* DrawHelpers::DrawCandidateGrasps(std::vector<CandidateGraspPtr>& cg, float scaleValue)
    {
        SoSeparator* sepCandidates = new SoSeparator;
        sepCandidates->ref();

        for (size_t i = 0; i < cg.size(); i++)
        {
            SoSeparator* sepCurrentCandidate =
                DrawHelpers::DrawCandidateGrasp(cg.at(i),
                                                scaleValue);
            sepCandidates->addChild(sepCurrentCandidate);
        }

        return sepCandidates;
    }

    Eigen::Vector3f DrawHelpers::getColorFromTable(float currentValue, float kFactor, float maxInputValue)
    {
        //kFactor: choose values of maybe 1.1 or 1.2 to avoid spheres of max radius to be drawn white...

        //Maximum value to be handled by this color table
        float maxColorValue = kFactor * maxInputValue;

        float R;
        float G;
        float B;

        if ((currentValue >= 0) && (currentValue <= 0.25 * maxColorValue))
        {
            R = 4 * (currentValue / maxColorValue);
            G = 0;
            B = 0;
        }
        else if ((currentValue > 0.25 * maxColorValue) && (currentValue <= 0.5 * maxColorValue))
        {
            R = -4 * (currentValue / maxColorValue) + 2;
            G = 4 * (currentValue / maxColorValue) - 1;
            B = 0;
        }
        else if ((currentValue > 0.5 * maxColorValue) && (currentValue <= 0.75 * maxColorValue))
        {
            R = 0;
            G = -4 * (currentValue / maxColorValue) + 3;
            B = 4 * (currentValue / maxColorValue) - 2;
        }
        else if ((currentValue > 0.75 * maxColorValue) && (currentValue <= maxColorValue))
        {
            R = 0;
            G = 0;
            B = -4 * (currentValue / maxColorValue) + 4;
        }
        else
        {
            VR_ERROR << "getColorFromTable(): OUTSIDE expected range of values!: "
                     << currentValue << endl;
            R = 0.7f;
            G = 0;
            B = 0.7f;
        }

        Eigen::Vector3f color(R, G, B);
        return color;
    }

}
