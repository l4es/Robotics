/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/

#include "OSGVisualizationFactory.h"
#include "../VisualizationNode.h"
#include "OSGVisualizationNode.h"
#include "../../VirtualRobotException.h"
#include "OSGVisualization.h"
#include "../../Robot.h"
#include "../../Grasping/Grasp.h"
#include "../../Grasping/GraspSet.h"
#include "../../SceneObject.h"
#include "../TriMeshModel.h"
#include "../../Workspace/Reachability.h"
#include <iostream>
#include <algorithm>

#include <osgDB/ReadFile>
#include <osg/ShapeDrawable>
#include <osg/BoundingBox>
#include <osg/PolygonMode>
#include <osg/ComputeBoundsVisitor>
#include <osg/LineWidth>
namespace VirtualRobot
{

    OSGVisualizationFactory::OSGVisualizationFactory()
    {
    }


    OSGVisualizationFactory::~OSGVisualizationFactory()
    {
    }


    /**
     * This method creates a VirtualRobot::OSGVisualizationNode from a given \p filename.
     * An instance of VirtualRobot::VisualizationNode is returned in case of an occured error.
     *
     * \param filename file to load the OSG3D visualization from.
     * \param boundingBox Use bounding box instead of full model.
     * \return instance of VirtualRobot::OSGVisualizationNode upon success and VirtualRobot::VisualizationNode on error.
     */
    VisualizationNodePtr OSGVisualizationFactory::getVisualizationFromFile(const std::string& filename, bool boundingBox)
    {
        VisualizationNodePtr visualizationNode(new VisualizationNode);

        osg::Node* n = osgDB::readNodeFile(filename.c_str());

        if (n)
        {
            n->ref();

            if (boundingBox)
            {
                osg::Node* bboxVisu = CreateBoundingBox(n);
                bboxVisu->ref();
                n->unref();
                n = bboxVisu;
            }

            visualizationNode.reset(new OSGVisualizationNode(n));

            visualizationNode->setFilename(filename, boundingBox);
            n->unref();
        }
        else
        {
            VR_WARNING << "Could not read file:" << filename << endl;
        }


        return visualizationNode;
    }


    /**
     * register this class in the super class factory
     */
    VisualizationFactory::SubClassRegistry OSGVisualizationFactory::registry(OSGVisualizationFactory::getName(), &OSGVisualizationFactory::createInstance);


    /**
     * \return "osg"
     */
    std::string OSGVisualizationFactory::getName()
    {
        return "osg";
    }


    /**
     * \return new instance of OSGVisualizationFactory
     */
    boost::shared_ptr<VisualizationFactory> OSGVisualizationFactory::createInstance(void*)
    {
        boost::shared_ptr<OSGVisualizationFactory> OSGFactory(new OSGVisualizationFactory());
        return OSGFactory;
    }

    VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createBox(float width, float height, float depth, float colorR, float colorG, float colorB)
    {
        osg::Box* b = new osg::Box(osg::Vec3(0, 0, 0), width, height, depth);
        osg::ShapeDrawable* bd = new osg::ShapeDrawable(b);
        bd->setColor(osg::Vec4(colorR, colorG, colorB, 1.0));
        osg::Geode* bg = new osg::Geode();
        bg->addDrawable(bd);

        osg::Group* s = new osg::Group;
        s->addChild(bg);

        VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
        return visualizationNode;
    }



    VisualizationNodePtr OSGVisualizationFactory::createLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width, float colorR, float colorG, float colorB)
    {
        osg::Vec3 sp(from(0, 3), from(1, 3), from(2, 3));
        osg::Vec3 ep(to(0, 3), to(1, 3), to(2, 3));
        osg::ref_ptr<osg::Geometry> beam(new osg::Geometry);
        osg::ref_ptr<osg::Vec3Array> points = new osg::Vec3Array;
        points->push_back(sp);
        points->push_back(ep);
        osg::ref_ptr<osg::Vec4Array> color = new osg::Vec4Array;
        color->push_back(osg::Vec4(colorR, colorG, colorB, 1.0));
        beam->setVertexArray(points.get());
        beam->setColorArray(color.get());
        beam->setColorBinding(osg::Geometry::BIND_PER_VERTEX); // BIND_PER_PRIMITIVE
        beam->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 2));


        osg::Geode* bg = new osg::Geode();
        bg->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        bg->addDrawable(beam);
        osg::Group* s = new osg::Group;
        s->addChild(bg);

        VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
        return visualizationNode;
    }

    VisualizationNodePtr OSGVisualizationFactory::createSphere(float radius, float colorR, float colorG, float colorB)
    {
        osg::Sphere* b = new osg::Sphere(osg::Vec3(0, 0, 0), radius);
        osg::ShapeDrawable* bd = new osg::ShapeDrawable(b);
        bd->setColor(osg::Vec4(colorR, colorG, colorB, 1.0));
        osg::Geode* bg = new osg::Geode();
        bg->addDrawable(bd);
        osg::Group* s = new osg::Group;
        s->addChild(bg);

        VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
        return visualizationNode;
    }

    VisualizationNodePtr OSGVisualizationFactory::createCoordSystem(float scaling, std::string* text, float axisLength, float axisSize, int nrOfBlocks)
    {
        osg::Node* s = OSGVisualizationFactory::CreateCoordSystemVisualization(scaling, text, axisLength, axisSize, nrOfBlocks);
        VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
        return visualizationNode;
    }



    osg::Node* OSGVisualizationFactory::CreateCoordSystemVisualization(float scaling, std::string* text, float axisLength, float axisSize, int nrOfBlocks)
    {

        osg::Node* xAxis = CreateArrow(Eigen::Vector3f(1, 0, 0), axisLength, axisSize, Color::Red());
        osg::Node* yAxis = CreateArrow(Eigen::Vector3f(0, 1, 0), axisLength, axisSize, Color::Green());
        osg::Node* zAxis = CreateArrow(Eigen::Vector3f(0, 0, 1), axisLength, axisSize, Color::Blue());

        osg::Group* g = new osg::Group;
        g->addChild(xAxis);
        g->addChild(yAxis);
        g->addChild(zAxis);
        return g;
        /*float blockSize = axisSize+0.5f;
        float blockWidth = 0.1f;
        if (axisSize>10.0f)
        {
            blockSize += axisSize / 10.0f;
            blockWidth += axisSize / 10.0f;
        }

        float axisBlockTranslation;
        if (nrOfBlocks!=0)
        {
            axisBlockTranslation = axisLength / nrOfBlocks;
        } else
            axisBlockTranslation = axisLength / 10.0f;

        SoSeparator* result = new SoSeparator();

        SbMatrix m;
        m.makeIdentity();
        SoMatrixTransform *mtr = new SoMatrixTransform();
        mtr->matrix.setValue(m);
        result->addChild(mtr);

        //SoScale *sc = new SoScale();
        //sc->scaleFactor.setValue(scaling,scaling,scaling);
        //result->addChild(sc);

        for (int i=0;i<3;i++)
        {
            SoSeparator *tmp1 = new SoSeparator();
            SoTransform *t = new SoTransform();
            SoMaterial *m = new SoMaterial();
            if (i==0)
            {
                m->diffuseColor.setValue(1.0f,0,0);
                t->translation.setValue((axisLength/2.0f + axisSize/2.0f)*scaling,0,0);
            } else if (i==1)
            {
                m->diffuseColor.setValue(0,1.0f,0);
                t->translation.setValue(0,(axisLength/2.0f + axisSize/2.0f)*scaling,0);
            } else
            {
                m->diffuseColor.setValue(0,0,1.0f);
                t->translation.setValue(0,0,(axisLength/2.0f + axisSize/2.0f)*scaling);
            }

            tmp1->addChild(m);
            tmp1->addChild(t);
            SoCube *c = new SoCube();
            SoCube *c2 = new SoCube();
            SoTransform *t2 = new SoTransform();
            if (i==0)
            {
                c->width = axisLength*scaling;
                c->height = axisSize*scaling;
                c->depth = axisSize*scaling;
                c2->width = blockWidth*scaling;
                c2->height = blockSize*scaling;
                c2->depth = blockSize*scaling;
                t2->translation.setValue(axisBlockTranslation*scaling,0,0);
            } else if (i==1)
            {
                c->height = axisLength*scaling;
                c->width = axisSize*scaling;
                c->depth = axisSize*scaling;
                c2->width = blockSize*scaling;
                c2->height = blockWidth*scaling;
                c2->depth = blockSize*scaling;
                t2->translation.setValue(0,axisBlockTranslation*scaling,0);
            } else
            {
                c->depth = axisLength*scaling;
                c->height = axisSize*scaling;
                c->width = axisSize*scaling;
                c2->width = blockSize*scaling;
                c2->height = blockSize*scaling;
                c2->depth = blockWidth*scaling;
                t2->translation.setValue(0,0,axisBlockTranslation*scaling);
            }
            tmp1->addChild(c);
            result->addChild(tmp1);

            SoSeparator *tmp2 = new SoSeparator();
            SoMaterial *m2 = new SoMaterial();
            m2->diffuseColor.setValue(1.0f,1.0f,1.0f);
            tmp2->addChild(m2);

            for (int j=0;j<nrOfBlocks;j++)
            {
                tmp2->addChild(t2);
                tmp2->addChild(c2);
            }

            result->addChild(tmp2);
        }

        if (text!=NULL)
        {
            SoSeparator *textSep = new SoSeparator();
            SoTranslation *moveT = new SoTranslation();
            moveT->translation.setValue(2.0f,2.0f,0.0f);
            textSep->addChild(moveT);
            SoAsciiText *textNode = new SoAsciiText();
            /*std::string text2(*text);
            text2.replace( ' ', "_" );* /
            SbString text2(text->c_str());
            text2.apply( &IVToolsHelper_ReplaceSpaceWithUnderscore );
            textNode->string.set(text2.getString());
            textSep->addChild(textNode);
            result->addChild(textSep);
        }
        return result;*/
    }
    /*VisualizationNodePtr OSGVisualizationFactory::createVisualization(CollisionCheckerPtr colChecker)
    {
        VR_INFO << "init nyi..." << endl;
        return VisualizationNodePtr();
    }*/



    VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createVertexVisualization(const Eigen::Vector3f& position, float radius, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/)
    {
        VR_INFO << "init nyi..." << endl;
        return VisualizationNodePtr();
    }

    VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createPlane(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, float extend, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/)
    {
        VR_INFO << "init nyi..." << endl;
        return VisualizationNodePtr();
    }

    VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createTrajectory(TrajectoryPtr t, Color colorNode, Color colorLine, float nodeSize, float lineSize)
    {
        VR_INFO << "init nyi..." << endl;
        return VisualizationNodePtr();
    }

    void OSGVisualizationFactory::switchToWireframe(osg::Node* srcNode)
    {
        if (!srcNode)
        {
            return;
        }

        osg::StateSet* state = srcNode->getOrCreateStateSet();
        osg::PolygonMode* polyModeObj;

        polyModeObj = dynamic_cast< osg::PolygonMode* >
                      (state->getAttribute(osg::StateAttribute::POLYGONMODE));

        if (!polyModeObj)
        {
            polyModeObj = new osg::PolygonMode;
            state->setAttribute(polyModeObj);
        }

        polyModeObj->setMode(osg::PolygonMode::FRONT_AND_BACK, osg::PolygonMode::LINE);
    }

    osg::Node* OSGVisualizationFactory::CreateBoundingBox(osg::Node* model, bool wireFrame)
    {
        if (!model)
        {
            return NULL;
        }

        const osg::MatrixList& m = model->getWorldMatrices();
        osg::ComputeBoundsVisitor cbv;
        model->accept(cbv);
        osg::BoundingBox bboxOSG = cbv.getBoundingBox();
        osg::Vec3 minV = bboxOSG._min * m.front();
        osg::Vec3 maxV = bboxOSG._max * m.front();

        BoundingBox bbox;
        Eigen::Vector3f minPoint(minV[0], minV[1], minV[2]);
        Eigen::Vector3f maxPoint(maxV[0], maxV[1], maxV[2]);
        bbox.addPoint(minPoint);
        bbox.addPoint(maxPoint);
        return CreateBoundingBoxVisualization(bbox, wireFrame);
    }

    osg::Node* OSGVisualizationFactory::CreateBoundingBoxVisualization(const BoundingBox& bbox, bool wireFrame)
    {
        osg::BoundingBox bboxOSG;

        bboxOSG.expandBy(bbox.getMin()(0), bbox.getMin()(1), bbox.getMin()(2));
        bboxOSG.expandBy(bbox.getMax()(0), bbox.getMax()(1), bbox.getMax()(2));

        osg::Vec3 ext(bboxOSG._max - bboxOSG._min);
        osg::Box* box = new osg::Box(bboxOSG.center(), ext[0], ext[1], ext[2]);
        osg::ShapeDrawable* shapeDraw = new osg::ShapeDrawable(box);
        osg::Geode* boundingBoxGeode = new osg::Geode();
        boundingBoxGeode->addDrawable(shapeDraw);

        if (wireFrame)
        {
            switchToWireframe(boundingBoxGeode);
        }

        return boundingBoxGeode;
    }

    VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createBoundingBox(const BoundingBox& bbox, bool wireFrame)
    {
        osg::Node* res = CreateBoundingBoxVisualization(bbox, wireFrame);

        VisualizationNodePtr node(new OSGVisualizationNode(res));
        return node;
    }

    osg::MatrixTransform* OSGVisualizationFactory::getMatrixTransform(const Eigen::Matrix4f& pose)
    {
        osg::MatrixTransform* mt = new osg::MatrixTransform;
        osg::Matrix mat(pose.data());
        mt->setMatrix(mat);
        return mt;
    }

    VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createTriMeshModelVisualization(TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f& pose)
    {
        osg::Group* res = new osg::Group;

        osg::MatrixTransform* globalPoseTransform = getMatrixTransform(pose);
        res->addChild(globalPoseTransform);

        osg::Node* res1 = OSGVisualizationFactory::getOSGVisualization(model, showNormals);
        globalPoseTransform->addChild(res1);

        VisualizationNodePtr node(new OSGVisualizationNode(res));
        return node;
    }

    osg::Node* OSGVisualizationFactory::getOSGVisualization(TriMeshModelPtr model, bool showNormals, VisualizationFactory::Color color)
    {
        osg::Group* res = new osg::Group;
        res->ref();
        Eigen::Vector3f v1, v2, v3;

        for (size_t i = 0; i < model->faces.size(); i++)
        {
            v1 = model->vertices[model->faces[i].id1];
            v2 = model->vertices[model->faces[i].id2];
            v3 = model->vertices[model->faces[i].id3];
            //v2.setValue(model->vertices[model->faces[i].id2](0),model->vertices[model->faces[i].id2](1),model->vertices[model->faces[i].id2](2));
            //v3.setValue(model->vertices[model->faces[i].id3](0),model->vertices[model->faces[i].id3](1),model->vertices[model->faces[i].id3](2));
            std::vector<Eigen::Vector3f> v;
            v.push_back(v1);
            v.push_back(v2);
            v.push_back(v3);
            osg::Node* s = CreatePolygonVisualization(v, color);
            res->addChild(s);

            if (showNormals)
            {
                v1 = (v1 + v2 + v3) / 3.0f;
                osg::Group* ar = new osg::Group;
                Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
                mat.block(0, 3, 3, 1) = v1;
                osg::MatrixTransform* mt = getMatrixTransform(mat);
                ar->addChild(mt);

                osg::Node* n = CreateArrow(model->faces[i].normal, 30.0f, 1.5f);
                mt->addChild(n);

                res->addChild(ar);
            }
        }

        res->unref_nodelete();
        return res;
    }

    osg::Node* OSGVisualizationFactory::CreatePolygonVisualization(const std::vector<Eigen::Vector3f>& points, VisualizationFactory::Color colorInner /*= VisualizationFactory::Color::Blue()*/, VisualizationFactory::Color colorLine /*= VisualizationFactory::Color::Black()*/, float lineSize /*= 5.0f*/)
    {
        osg::Geode* geode = new osg::Geode();
        // create Geometry object to store all the vertices and lines primitive.
        osg::Geometry* polyGeom = new osg::Geometry();
        osg::Vec3Array* vertices = new osg::Vec3Array;
        Eigen::Vector3f normal = MathTools::findNormal(points);

        for (unsigned int i = 0; i < points.size(); i++)
        {
            vertices->push_back(osg::Vec3(points[i](0), points[i](1), points[i](2)));

        }

        int numCoords = int(points.size());

        // pass the created vertex array to the points geometry object.
        polyGeom->setVertexArray(vertices);

        osg::Vec4Array* colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(colorInner.r, colorInner.g, colorInner.b, 1.0f - colorInner.transparency));
        polyGeom->setColorArray(colors);
        polyGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

        osg::Vec3Array* normals = new osg::Vec3Array;
        normals->push_back(osg::Vec3(-normal(0), -normal(1), -normal(2)));
        polyGeom->setNormalArray(normals);
        polyGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

        polyGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POLYGON, 0, numCoords));
        //printTriangles("Polygon",*polyGeom);
        geode->addDrawable(polyGeom);


        osg::Geometry* linesGeom = new osg::Geometry();
        linesGeom->setVertexArray(vertices);
        osg::Vec4Array* colorsL = new osg::Vec4Array;
        colorsL->push_back(osg::Vec4(colorLine.r, colorLine.g, colorLine.b, 1.0f - colorLine.transparency));
        linesGeom->setColorArray(colorsL);
        linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);
        linesGeom->setNormalArray(normals);
        linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);
        linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, numCoords));
        osg::LineWidth* linewidth = new osg::LineWidth();
        linewidth->setWidth(lineSize);
        geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);
        geode->addDrawable(linesGeom);

        return geode;
    }
    VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createArrow(const Eigen::Vector3f& n, float length /*= 50.0f*/, float width /*= 2.0f*/, const Color& color /*= Color::Gray()*/)
    {
        osg::Node* res = CreateArrow(n, length, width, color);

        VisualizationNodePtr node(new OSGVisualizationNode(res));
        return node;
    }

    osg::Node* OSGVisualizationFactory::CreateArrow(const Eigen::Vector3f& n, float length /*= 50.0f*/, float width /*= 2.0f*/, const Color& color /*= Color::Gray()*/)
    {
        float coneHeight = width * 6.0f;
        float coneBotomRadius = width * 2.5f;
        osg::Group* res = new osg::Group;

        osg::Vec3 objNormal(n(0), n(1), n(2));
        osg::Matrix objNormalTrafo;
        objNormalTrafo.makeIdentity();

        osg::MatrixTransform* arrow = new osg::MatrixTransform;

        // Rotate X-axis arrow appropriately.
        osg::Quat rotation;
        rotation.makeRotate(osg::Vec3(0, 0, 1.0), objNormal);
        arrow->setMatrix(osg::Matrix(rotation));

        res->addChild(arrow);

        osg::Geode* geodeCyl = new osg::Geode;
        osg::Cylinder* cyl = new osg::Cylinder(osg::Vec3(0, 0, length * 0.5), width, length);
        osg::ShapeDrawable* cylDraw = new osg::ShapeDrawable(cyl);

        if (!color.isNone())
        {
            cylDraw->setColor(osg::Vec4(color.r, color.g, color.b, 1.0 - color.transparency));
        }

        geodeCyl->addDrawable(cylDraw);
        arrow->addChild(geodeCyl);

        osg::Geode* geodeCone = new osg::Geode;
        osg::Cone* cone = new osg::Cone(osg::Vec3(0.0f, 0, length), coneBotomRadius, coneHeight);
        osg::ShapeDrawable* coneDraw = new osg::ShapeDrawable(cone);

        if (!color.isNone())
        {
            coneDraw->setColor(osg::Vec4(color.r, color.g, color.b, 1.0 - color.transparency));
        }

        geodeCone->addDrawable(coneDraw);
        arrow->addChild(geodeCone);

        return res;
    }

    VirtualRobot::VisualizationNodePtr OSGVisualizationFactory::createVisualization()
    {
        osg::Group* s = new osg::Group;
        VisualizationNodePtr visualizationNode(new OSGVisualizationNode(s));
        return visualizationNode;
    }

    osg::Matrix* OSGVisualizationFactory::getRelativePose(osg::Node* n, osg::Node* rootNode)
    {
        globalPoseNodeVisitor* ncv = new globalPoseNodeVisitor(rootNode);

        if (n && ncv)
        {
            n->accept(*ncv);
            return ncv->getGlobalPose();
        }

        osg::Matrix* resId = new osg::Matrix;
        resId->makeIdentity();
        return resId;
    }

} // namespace VirtualRobot
