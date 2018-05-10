/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @author     Manfred Kroehnert
* @copyright  2010,2011 Nikolaus Vahrenkamp, Manfred Kroehnert
*/
#include "CoinVisualizationFactory.h"
#include "../VisualizationNode.h"
#include "CoinVisualizationNode.h"
#include "../../VirtualRobotException.h"
#include "../../RuntimeEnvironment.h"
#include "CoinVisualization.h"
#include "../../Robot.h"
#include "../../Grasping/Grasp.h"
#include "../../Trajectory.h"
#include "../../Grasping/GraspSet.h"
#include "../../SceneObject.h"
#include "../../IK/constraints/TSRConstraint.h"
#include "../../IK/constraints/BalanceConstraint.h"
#include "../../IK/constraints/PoseConstraint.h"
#include "../../IK/SupportPolygon.h"
#include "../TriMeshModel.h"
#include "../../Workspace/Reachability.h"
#include "../../Workspace/WorkspaceGrid.h"
#include "../../XML/BaseIO.h"
#include "../../Import/MeshImport/STLReader.h"
#include <Inventor/SoDB.h>
#include <Inventor/nodes/SoFile.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/nodes/SoUnits.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoCone.h>
#include <Inventor/nodes/SoRotationXYZ.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoAsciiText.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoDirectionalLight.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/SbViewportRegion.h>
#include <Inventor/actions/SoGetBoundingBoxAction.h>
#include <Inventor/nodes/SoComplexity.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoTextureCoordinateBinding.h>
#include <Inventor/nodes/SoTexture2.h>
#include <Inventor/nodes/SoTextureCoordinate2.h>
#include <Inventor/nodes/SoNormal.h>
#include <Inventor/nodes/SoFaceSet.h>
#include <Inventor/nodes/SoCylinder.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/VRMLnodes/SoVRMLBillboard.h>
#include <Inventor/nodes/SoShapeHints.h>
#include <Inventor/nodes/SoLightModel.h>
#include <Inventor/actions/SoSearchAction.h>
#include <iostream>
#include <algorithm>


namespace VirtualRobot
{

    CoinVisualizationFactory::CoinVisualizationFactory()
    {
    }


    CoinVisualizationFactory::~CoinVisualizationFactory()
    {
    }

    /**
    * This method creates a VirtualRobot::CoinVisualizationNode from a given vector of \p primitives.
    * An instance of VirtualRobot::VisualizationNode is returned in case of an occured error.
    *
    * \param primitives vector of primitives to create the visualization from.
    * \param boundingBox Use bounding box instead of full model.
    * \return instance of VirtualRobot::CoinVisualizationNode upon success and VirtualRobot::VisualizationNode on error.
    */
    VisualizationNodePtr CoinVisualizationFactory::getVisualizationFromPrimitives(const std::vector<Primitive::PrimitivePtr>& primitives, bool boundingBox, Color color)
    {
        VisualizationNodePtr visualizationNode = VisualizationNodePtr(new VisualizationNode());
        SoSeparator* coinVisualization = new SoSeparator();
        coinVisualization->ref();

        Eigen::Matrix4f currentTransform = Eigen::Matrix4f::Identity();

        for (std::vector<Primitive::PrimitivePtr>::const_iterator it = primitives.begin(); it != primitives.end(); it++)
        {
            Primitive::PrimitivePtr p = *it;
            currentTransform *= p->transform;
            SoSeparator* soSep = new SoSeparator();
            SoNode* pNode = GetNodeFromPrimitive(p, boundingBox, color);
            soSep->addChild(getMatrixTransformScaleMM2M(currentTransform));
            soSep->addChild(pNode);
            coinVisualization->addChild(soSep);
        }

        if (boundingBox)
        {
            SoSeparator* bboxVisu = CreateBoundingBox(coinVisualization, false);
            bboxVisu->ref();
            coinVisualization->unref();
            coinVisualization = bboxVisu;
        }

        // create new CoinVisualizationNode if no error occurred
        visualizationNode.reset(new CoinVisualizationNode(coinVisualization));
        visualizationNode->primitives = primitives;

        coinVisualization->unref();

        return visualizationNode;
    }

    SoNode* CoinVisualizationFactory::GetNodeFromPrimitive(Primitive::PrimitivePtr primitive, bool boundingBox, Color color)
    {
        SoSeparator* coinVisualization = new SoSeparator;
        SoNode* c = getColorNode(color);
        coinVisualization->addChild(c);

        if (primitive->type == Primitive::Box::TYPE)
        {
            Primitive::Box* box = boost::dynamic_pointer_cast<Primitive::Box>(primitive).get();
            SoCube* soBox = new SoCube;
            soBox->width = box->width / 1000.f;
            soBox->height = box->height / 1000.f;
            soBox->depth = box->depth / 1000.f;
            coinVisualization->addChild(soBox);
        }
        else if (primitive->type == Primitive::Sphere::TYPE)
        {
            Primitive::Sphere* sphere = boost::dynamic_pointer_cast<Primitive::Sphere>(primitive).get();
            SoSphere* soSphere = new SoSphere;
            soSphere->radius = sphere->radius / 1000.f;
            coinVisualization->addChild(soSphere);
        }
        else if (primitive->type == Primitive::Cylinder::TYPE)
        {
            Primitive::Cylinder* cylinder = boost::dynamic_pointer_cast<Primitive::Cylinder>(primitive).get();
            SoCylinder* soCylinder = new SoCylinder;
            soCylinder->radius = cylinder->radius / 1000.f;
            soCylinder->height = cylinder->height / 1000.f;
            coinVisualization->addChild(soCylinder);
        }

        if (boundingBox && coinVisualization)
        {
            SoSeparator* bboxVisu = CreateBoundingBox(coinVisualization, false);
            coinVisualization->addChild(bboxVisu);
        }

        return coinVisualization;
    }

    /**
    * This method creates a VirtualRobot::CoinVisualizationNode from a given \p filename.
    * An instance of VirtualRobot::VisualizationNode is returned in case of an occured error.
    *
    * \param filename file to load the Coin3D visualization from.
    * \param boundingBox Use bounding box instead of full model.
    * \return instance of VirtualRobot::CoinVisualizationNode upon success and VirtualRobot::VisualizationNode on error.
    */
    VisualizationNodePtr CoinVisualizationFactory::getVisualizationFromFile(const std::string& filename, bool boundingBox)
    {
        // passing an empty string to SoInput and trying to open it aborts the program
        if (filename.empty())
        {
            std::cerr <<  "No filename given" << std::endl;
            return VisualizationNodePtr();
        }

        // check for STL file (.stl, .stla, .stlb)
        if (filename.length() >= 4)
        {
            std::string ending = filename.substr(filename.length() - 4, 4);
            BaseIO::getLowerCase(ending);

            if (ending == ".stl" || ending == "stla" || ending == "stlb")
            {
                return getVisualizationFromSTLFile(filename, boundingBox);
            }
        }

        return getVisualizationFromCoin3DFile(filename, boundingBox);
    }

    VisualizationNodePtr CoinVisualizationFactory::getVisualizationFromCoin3DFile(const std::string& filename, bool boundingBox)
    {
        VisualizationNodePtr visualizationNode(new VisualizationNode);
        // try to open the given file
        SoInput fileInput;

        if (!fileInput.openFile(filename.c_str()))
        {
            std::cerr <<  "Cannot open file " << filename << std::endl;
            return visualizationNode;
        }

        CoinVisualizationFactory::GetVisualizationFromSoInput(fileInput, visualizationNode, boundingBox);

        fileInput.closeFile();
        visualizationNode->setFilename(filename, boundingBox);

        return visualizationNode;
    }

    VisualizationNodePtr CoinVisualizationFactory::getVisualizationFromSTLFile(const std::string& filename, bool boundingBox)
    {
        VisualizationNodePtr visualizationNode(new VisualizationNode);
        // try to read from file
        visualizationNode->setFilename(filename, boundingBox);

        TriMeshModelPtr t(new TriMeshModel());
        STLReaderPtr r(new STLReader());
        r->setScaling(1000.0f); // mm
        bool readOK = r->read(filename, t);

        if (!readOK)
        {
            VR_ERROR << "Could not read stl file " << filename << endl;
            return visualizationNode;
        }

        Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
        visualizationNode = createTriMeshModelVisualization(t, id);

        return visualizationNode;
    }

    VisualizationNodePtr CoinVisualizationFactory::getVisualizationFromFile(const std::ifstream& ifs, bool boundingBox)
    {
        VisualizationNodePtr visualizationNode(new VisualizationNode);

        // passing an empty string to SoInput and trying to open it aborts the program
        if (!ifs)
        {
            std::cerr <<  "Filestream not valid" << std::endl;
            return visualizationNode;
        }

        // try to open the given file
        std::ostringstream oss;
        oss << ifs.rdbuf();

        if (!ifs && !ifs.eof())
        {
            std::cerr <<  "Error reading filestream " << std::endl;
            return visualizationNode;
        }

        std::string contents(oss.str());
        return getVisualizationFromString(contents);
    }

    /**
    * This method creates a VirtualRobot::CoinVisualizationNode from a given \p modelString.
    * An instance of VirtualRobot::VisualizationNode is returned in case of an occured error.
    *
    * \param modelString string to load the Coin3D visualization from.
    * \param boundingBox Use bounding box instead of full model.
    * \return instance of VirtualRobot::CoinVisualizationNode upon succes and VirtualRobot::VisualizationNode on error.
    */
    VisualizationNodePtr CoinVisualizationFactory::getVisualizationFromString(const std::string& modelString, bool boundingBox)
    {
        VisualizationNodePtr visualizationNode(new VisualizationNode);

        if (modelString.empty())
        {
            std::cerr << "No modelString given" << std::endl;
            return visualizationNode;
        }

        SoInput stringInput;
        stringInput.setBuffer(const_cast<char*>(modelString.c_str()), modelString.size());
        CoinVisualizationFactory::GetVisualizationFromSoInput(stringInput, visualizationNode, boundingBox);
        return visualizationNode;
    }


    /**
    * This method reads the data from the given \p soInput and creates a new CoinVisualizationNode
    * with the read Coin model if no error occurred during reading the model.
    * The newly created CoinVisualizationNode is then stored in \p visualisationNode.
    *
    * \param soInput SoInput instance from which the model is read.
    * \param visualizationNode VisualizationNodePtr instance in which the created CoinVisualizationNode is stored.
    * \param boundingBox Use bounding box instead of full model.
    */
    void CoinVisualizationFactory::GetVisualizationFromSoInput(SoInput& soInput, VisualizationNodePtr& visualizationNode, bool boundingBox)
    {
        // read the contents of the file
        SoNode* coinVisualization = SoDB::readAll(&soInput);

        // check if the visualization was read
        if (NULL == coinVisualization)
        {
            std::cerr <<  "Problem reading model from SoInput: "  << soInput.getCurFileName() << std::endl;
            return;
        }

        coinVisualization->ref();

        if (boundingBox)
        {
            SoSeparator* bboxVisu = CreateBoundingBox(coinVisualization, false);
            bboxVisu->ref();
            coinVisualization->unref();
            coinVisualization = bboxVisu;
        }

        // create new CoinVisualizationNode if no error occurred
        visualizationNode.reset(new CoinVisualizationNode(coinVisualization));

        coinVisualization->unref();
    }



    SoSeparator* CoinVisualizationFactory::CreateBoundingBox(SoNode* ivModel, bool wireFrame)
    {
        THROW_VR_EXCEPTION_IF(!ivModel, "NULL ivModel!");

        float minX;
        float minY;
        float minZ;
        float maxX;
        float maxY;
        float maxZ;

        // get dimensions of oivMod
        SbViewportRegion vpr;
        SoGetBoundingBoxAction boxAction(vpr);
        boxAction.apply(ivModel);

        //boxAction.getXfBoundingBox().getBounds(minX, minY, minZ, maxX, maxY, maxZ);
        boxAction.getBoundingBox().getBounds(minX, minY, minZ, maxX, maxY, maxZ);
        cout << "x: " << minX << "," << maxX << " ; Y: " << minY << "," << maxY << " ; Z: " << minZ << "," << maxZ << endl;


        SoCube* cu = new SoCube();

        cu->width = (maxX - minX);
        cu->height = (maxY - minY);
        cu->depth = (maxZ - minZ);


        SoDrawStyle* s = new SoDrawStyle();

        if (wireFrame)
        {
            s->style =  SoDrawStyle::LINES;
        }
        else
        {
            s->style =  SoDrawStyle::FILLED;
        }

        SoSeparator* n = new SoSeparator();
        SoTranslation* t = new SoTranslation();
        t->translation.setValue((maxX - minX) * 0.5f + minX, (maxY - minY) * 0.5f + minY, (maxZ - minZ) * 0.5f + minZ);
        n->addChild(t);
        n->addChild(s);
        n->addChild(cu);
        return n;
    }


    /**
    * register this class in the super class factory
    */
    VisualizationFactory::SubClassRegistry CoinVisualizationFactory::registry(CoinVisualizationFactory::getName(), &CoinVisualizationFactory::createInstance);


    /**
    * \return "inventor"
    */
    std::string CoinVisualizationFactory::getName()
    {
        return "inventor";
    }


    /**
    * \return new instance of CoinVisualizationFactory and call SoDB::init()
    * if it has not already been called.
    */
    boost::shared_ptr<VisualizationFactory> CoinVisualizationFactory::createInstance(void*)
    {
        if (!SoDB::isInitialized())
        {
            SoDB::init();
        }

        boost::shared_ptr<CoinVisualizationFactory> coinFactory(new CoinVisualizationFactory());
        return coinFactory;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createBox(float width, float height, float depth, float colorR, float colorG, float colorB)
    {
        SoSeparator* s = new SoSeparator();
        s->ref();

        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        s->addChild(u);

        SoMaterial* m = new SoMaterial();
        s->addChild(m);
        m->ambientColor.setValue(colorR, colorG, colorB);
        m->diffuseColor.setValue(colorR, colorG, colorB);

        SoCube* c = new SoCube();
        s->addChild(c);
        c->width = width;
        c->height = height;
        c->depth = depth;

        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        s->unref();
        return visualizationNode;
    }

    SoNode* CoinVisualizationFactory::createCoinLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width, float colorR, float colorG, float colorB)
    {
        SoSeparator* s = new SoSeparator();
        s->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        s->addChild(u);

        SoMaterial* m = new SoMaterial();
        s->addChild(m);
        m->ambientColor.setValue(colorR, colorG, colorB);
        m->diffuseColor.setValue(colorR, colorG, colorB);

        // create line
        float x = from(0, 3);
        float y = from(1, 3);
        float z = from(2, 3);
        float x2 = to(0, 3);
        float y2 = to(1, 3);
        float z2 = to(2, 3);

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

    VisualizationNodePtr CoinVisualizationFactory::createLine(const Eigen::Matrix4f& from, const Eigen::Matrix4f& to, float width, float colorR, float colorG, float colorB)
    {
        SoNode* s = createCoinLine(from, to, width, colorR, colorG, colorB);
        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        return visualizationNode;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createLine(const Eigen::Vector3f& from, const Eigen::Vector3f& to, float width /*= 1.0f*/, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/)
    {
        Eigen::Matrix4f fromM;
        fromM.setIdentity();
        fromM.block(0, 3, 3, 1) = from;
        Eigen::Matrix4f toM;
        toM.setIdentity();
        toM.block(0, 3, 3, 1) = to;
        return createLine(from, to, width, colorR, colorG, colorB);
    }

    VisualizationNodePtr CoinVisualizationFactory::createSphere(float radius, float colorR, float colorG, float colorB)
    {
        SoSeparator* s = new SoSeparator();
        s->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        s->addChild(u);

        SoMaterial* m = new SoMaterial();
        s->addChild(m);
        m->ambientColor.setValue(colorR, colorG, colorB);
        m->diffuseColor.setValue(colorR, colorG, colorB);

        SoSphere* c = new SoSphere();
        s->addChild(c);
        c->radius = radius;

        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        s->unref();
        return visualizationNode;
    }

    VisualizationNodePtr CoinVisualizationFactory::createCylinder(float radius, float height, float colorR, float colorG, float colorB)
    {
        SoSeparator* s = new SoSeparator();
        s->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        s->addChild(u);

        SoMaterial* m = new SoMaterial();
        s->addChild(m);
        m->ambientColor.setValue(colorR, colorG, colorB);
        m->diffuseColor.setValue(colorR, colorG, colorB);

        SoCylinder* c = new SoCylinder();
        s->addChild(c);
        c->radius = radius;
        c->height = height;

        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        s->unref();
        return visualizationNode;
    }

    VisualizationNodePtr CoinVisualizationFactory::createCoordSystem(float scaling, std::string* text, float axisLength, float axisSize, int nrOfBlocks)
    {
        SoSeparator* s = CoinVisualizationFactory::CreateCoordSystemVisualization(scaling, text, axisLength, axisSize, nrOfBlocks);
        s->ref();

        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        s->unref();
        return visualizationNode;
    }

    VisualizationNodePtr CoinVisualizationFactory::createVisualization()
    {
        SoSeparator* s = new SoSeparator();
        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(s));
        return visualizationNode;
    }


    SoSeparator* CoinVisualizationFactory::CreateCoordSystemVisualization(float scaling, std::string* text, float axisLength, float axisSize, int nrOfBlocks)
    {
        float blockSize = axisSize + 0.5f;
        float blockWidth = 0.1f;

        if (axisSize > 10.0f)
        {
            blockSize += axisSize / 10.0f;
            blockWidth += axisSize / 10.0f;
        }

        float axisBlockTranslation;

        if (nrOfBlocks != 0)
        {
            axisBlockTranslation = axisLength / nrOfBlocks;
        }
        else
        {
            axisBlockTranslation = axisLength / 10.0f;
        }

        SoSeparator* result = new SoSeparator();
        result->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        result->addChild(u);

        SbMatrix m;
        m.makeIdentity();
        SoMatrixTransform* mtr = new SoMatrixTransform();
        mtr->matrix.setValue(m);
        result->addChild(mtr);

        //SoScale *sc = new SoScale();
        //sc->scaleFactor.setValue(scaling,scaling,scaling);
        //result->addChild(sc);

        for (int i = 0; i < 3; i++)
        {
            SoSeparator* tmp1 = new SoSeparator();
            SoTransform* t = new SoTransform();
            SoMaterial* m = new SoMaterial();

            if (i == 0)
            {
                m->diffuseColor.setValue(1.0f, 0, 0);
                t->translation.setValue((axisLength / 2.0f + axisSize / 2.0f)*scaling, 0, 0);
            }
            else if (i == 1)
            {
                m->diffuseColor.setValue(0, 1.0f, 0);
                t->translation.setValue(0, (axisLength / 2.0f + axisSize / 2.0f)*scaling, 0);
            }
            else
            {
                m->diffuseColor.setValue(0, 0, 1.0f);
                t->translation.setValue(0, 0, (axisLength / 2.0f + axisSize / 2.0f)*scaling);
            }

            tmp1->addChild(m);
            tmp1->addChild(t);
            SoCube* c = new SoCube();
            SoCube* c2 = new SoCube();
            SoTransform* t2 = new SoTransform();

            if (i == 0)
            {
                c->width = axisLength * scaling;
                c->height = axisSize * scaling;
                c->depth = axisSize * scaling;
                c2->width = blockWidth * scaling;
                c2->height = blockSize * scaling;
                c2->depth = blockSize * scaling;
                t2->translation.setValue(axisBlockTranslation * scaling, 0, 0);
            }
            else if (i == 1)
            {
                c->height = axisLength * scaling;
                c->width = axisSize * scaling;
                c->depth = axisSize * scaling;
                c2->width = blockSize * scaling;
                c2->height = blockWidth * scaling;
                c2->depth = blockSize * scaling;
                t2->translation.setValue(0, axisBlockTranslation * scaling, 0);
            }
            else
            {
                c->depth = axisLength * scaling;
                c->height = axisSize * scaling;
                c->width = axisSize * scaling;
                c2->width = blockSize * scaling;
                c2->height = blockSize * scaling;
                c2->depth = blockWidth * scaling;
                t2->translation.setValue(0, 0, axisBlockTranslation * scaling);
            }

            tmp1->addChild(c);
            result->addChild(tmp1);

            SoSeparator* tmp2 = new SoSeparator();
            SoMaterial* m2 = new SoMaterial();
            m2->diffuseColor.setValue(1.0f, 1.0f, 1.0f);
            tmp2->addChild(m2);

            for (int j = 0; j < nrOfBlocks; j++)
            {
                tmp2->addChild(t2);
                tmp2->addChild(c2);
            }

            result->addChild(tmp2);
        }

        if (text != NULL)
        {
            SoSeparator* textSep = new SoSeparator();
            SoTranslation* moveT = new SoTranslation();
            moveT->translation.setValue(2.0f, 2.0f, 0.0f);
            textSep->addChild(moveT);
            SoAsciiText* textNode = new SoAsciiText();
            /*std::string text2(*text);
            text2.replace( ' ', "_" );*/
            SbString text2(text->c_str());
            text2.apply(&IVToolsHelper_ReplaceSpaceWithUnderscore);
            textNode->string.set(text2.getString());
            textSep->addChild(textNode);
            result->addChild(textSep);
        }

        result->unrefNoDelete();
        return result;
    }

    VisualizationNodePtr CoinVisualizationFactory::createText(const std::string& text, bool billboard, float scaling, Color c, float offsetX, float offsetY, float offsetZ)
    {
        SoSeparator* res = new SoSeparator();
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);
        SoScale* sc = new SoScale();
        sc->scaleFactor.setValue(scaling, scaling, scaling);
        res->addChild(sc);
        SoMaterial* m = new SoMaterial();
        res->addChild(m);
        m->diffuseColor.setValue(c.r, c.g, c.b);
        m->transparency.setValue(c.transparency);

        if (billboard)
        {
            res->addChild(CreateBillboardText(text, offsetX * 1000.0f, offsetY * 1000.0f, offsetZ * 1000.0f));
        }
        else
        {
            res->addChild(CreateText(text, offsetX * 1000.0f, offsetY * 1000.0f, offsetZ * 1000.0f));
        }

        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(res));
        res->unref();
        return visualizationNode;
    }


    VisualizationNodePtr CoinVisualizationFactory::createEllipse(float x, float y, float z, bool showAxes, float axesHeight, float axesWidth)
    {
        SoSeparator* res = new SoSeparator();
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);
        res->addChild(CreateEllipse(x, y, z, NULL, showAxes, axesHeight, axesWidth));
        VisualizationNodePtr visualizationNode(new CoinVisualizationNode(res));
        res->unref();
        return visualizationNode;
    }

    SoSeparator* CoinVisualizationFactory::CreateText(const std::string& s, float offsetX, float offsetY, float offsetZ)
    {
        SoSeparator* textSep = new SoSeparator();
        textSep->ref();

        SoTranslation* moveT = new SoTranslation();
        textSep->addChild(moveT);
        moveT->translation.setValue(offsetX * 0.001f, offsetY * 0.001f, offsetZ * 0.001f);

        SoAsciiText* textNode = new SoAsciiText();
        textSep->addChild(textNode);
        /*std::string text2(*text);
        text2.replace( ' ', "_" );*/
        SbString text2(s.c_str());
        text2.apply(&IVToolsHelper_ReplaceSpaceWithUnderscore);
        textNode->string.set(text2.getString());

        textSep->unrefNoDelete();
        return textSep;
    }

    SoSeparator* CoinVisualizationFactory::CreateBillboardText(const std::string& s, float offsetX, float offsetY, float offsetZ)
    {
        SoSeparator* textSep = new SoSeparator();
        textSep->ref();

        SoTranslation* moveT = new SoTranslation();
        textSep->addChild(moveT);
        moveT->translation.setValue(offsetX * 0.001f, offsetY * 0.001f, offsetZ * 0.001f);

        SoVRMLBillboard* bb = new SoVRMLBillboard();
        textSep->addChild(bb);
        SoAsciiText* textNode = new SoAsciiText();
        bb->addChild(textNode);
        /*std::string text2(*text);
        text2.replace( ' ', "_" );*/
        SbString text2(s.c_str());
        text2.apply(&IVToolsHelper_ReplaceSpaceWithUnderscore);
        textNode->string.set(text2.getString());

        textSep->unrefNoDelete();
        return textSep;
    }

    SoSeparator* CoinVisualizationFactory::CreateVertexVisualization(const Eigen::Vector3f& position, float radius, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        // Control complexity  of the scene's primitives
        SoComplexity* comp = new SoComplexity;
        comp->value.setValue(0.1f);
        res->addChild(comp);

        // Set the vertex-position
        SoTranslation* t = new SoTranslation;
        t->translation.setValue(position(0), position(1), position(2));

        // Set material
        SoMaterial* m = new SoMaterial;
        m->transparency.setValue(transparency);


        m->diffuseColor.setValue(colorR, colorG, colorB);
        m->ambientColor.setValue(colorR, colorG, colorB);

        // Set shape
        SoSphere* s = new SoSphere;
        s->radius = radius;

        res->addChild(t);
        res->addChild(m);
        res->addChild(s);
        res->unrefNoDelete();
        return res;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createVertexVisualization(const Eigen::Vector3f& position, float radius, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/)
    {
        VisualizationNodePtr node(new CoinVisualizationNode(CreateVertexVisualization(position, radius, transparency, colorR, colorG, colorB)));
        return node;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createPlane(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, float extend, float transparency, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/)
    {
        SoSeparator* res = CreatePlaneVisualization(position, normal, extend, transparency, false, colorR, colorG, colorB);

        VisualizationNodePtr node(new CoinVisualizationNode(res));
        return node;
    }

    SoSeparator* CoinVisualizationFactory::CreatePlaneVisualization(const Eigen::Vector3f& position, const Eigen::Vector3f& normal, float extend, float transparency, bool grid, float colorR /*= 0.5f*/, float colorG /*= 0.5f*/, float colorB /*= 0.5f*/, std::string textureFile)
    {
        SoSeparator* res = new SoSeparator();
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        SoMatrixTransform* matrix = new SoMatrixTransform;
        SbMatrix mat;
        SbVec3f t(position(0), position(1), position(2));
        SbRotation r(SbVec3f(0, 0, 1.0f), SbVec3f(normal(0), normal(1), normal(2))); // rotateFrom, rotateTo
        mat.setTransform(t, r, SbVec3f(1.0f, 1.0f, 1.0f));
        matrix->matrix.setValue(mat);
        res->addChild(matrix);

        // Set material
        SoMaterial* m = new SoMaterial;
        m->transparency.setValue(transparency);
        m->diffuseColor.setValue(colorR, colorG, colorB);
        m->ambientColor.setValue(colorR, colorG, colorB);
        res->addChild(m);

        if (grid)
        {
            SoSeparator* res2;

            if (!textureFile.empty() && RuntimeEnvironment::getDataFileAbsolute(textureFile))
            {
                res2 = CreateGrid(extend, extend, extend / 500.0f, extend / 500.0f, true, textureFile.c_str(), transparency);
            }
            else
            {
                if (transparency == 0)
                {
                    std::string filename("images/FloorWhite.png");
                    RuntimeEnvironment::getDataFileAbsolute(filename);
                    res2 = CreateGrid(extend, extend, extend / 500.0f, extend / 500.0f, true, filename.c_str(), transparency);
                }
                else
                {
                    std::string filename("images/Floor.png");
                    RuntimeEnvironment::getDataFileAbsolute(filename);
                    res2 = CreateGrid(extend, extend, extend / 500.0f, extend / 500.0f, true, filename.c_str(), transparency);
                }
            }

            res->addChild(res2);
        }
        else
        {
            // Set shape
            SoCube* c = new SoCube;
            c->width = extend;
            c->depth = 1.0f; // Z
            c->height = extend;
            res->addChild(c);
        }

        res->unrefNoDelete();
        return res;
    }


    SoSeparator* CoinVisualizationFactory::CreateGrid(float width, float depth, float widthMosaic, float depthMosaic, bool InvertNormal, const char* pFileName, float Transparency)
    {
        SoSeparator* pGrid = new SoSeparator;
        pGrid->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        pGrid->addChild(u);
        SoMaterial* pSoMaterial = new SoMaterial;
        pSoMaterial->transparency = Transparency;
        float X = width / 2.0f;
        float Z = depth / 2.0f;
        SoCoordinate3* pImagePlaneSoCoordinate3 = new SoCoordinate3;
        pImagePlaneSoCoordinate3->point.set1Value(0, SbVec3f(X, Z, 0.0f));
        pImagePlaneSoCoordinate3->point.set1Value(1, SbVec3f(-X, Z, 0.0f));
        pImagePlaneSoCoordinate3->point.set1Value(2, SbVec3f(-X, -Z, 0.0f));
        pImagePlaneSoCoordinate3->point.set1Value(3, SbVec3f(X, -Z, 0.0f));
        /*SoNormal* pSoNormal = new SoNormal;
        pSoNormal->vector.set1Value(0, SbVec3f(0.0f, 0.0f, InvertNormal?-1.0f:1.0f));
        SoNormalBinding* pSoNormalBinding = new SoNormalBinding;
        pSoNormalBinding->value.setValue(SoNormalBinding::OVERALL);*/
        SoFaceSet* pSoFaceSet = new SoFaceSet;
        pSoFaceSet->numVertices.set1Value(0, 4);
        SoTextureCoordinate2* pSoTextureCoordinate2 = new SoTextureCoordinate2;
        pSoTextureCoordinate2->point.set1Value(0, SbVec2f(widthMosaic, 0));
        pSoTextureCoordinate2->point.set1Value(1, SbVec2f(0, 0));
        pSoTextureCoordinate2->point.set1Value(2, SbVec2f(0, depthMosaic));
        pSoTextureCoordinate2->point.set1Value(3, SbVec2f(widthMosaic, depthMosaic));
        SoTextureCoordinateBinding* pSoTextureCoordinateBinding =  new SoTextureCoordinateBinding;
        pSoTextureCoordinateBinding->value.setValue(SoTextureCoordinateBinding::PER_VERTEX);
        SoTexture2* pSoTexture2 = new SoTexture2;

        if (pFileName)
        {
            pSoTexture2->filename.setValue(pFileName);
        }

        pSoTexture2->wrapS = pSoTexture2->wrapT = SoTexture2::REPEAT;
        pGrid->addChild(pSoMaterial);
        pGrid->addChild(pSoTextureCoordinate2);
        pGrid->addChild(pSoTextureCoordinateBinding);
        pGrid->addChild(pSoTexture2);
        pGrid->addChild(pImagePlaneSoCoordinate3);
        //pGrid->addChild(pSoNormal);
        //pGrid->addChild(pSoNormalBinding);
        pGrid->addChild(pSoFaceSet);
        pGrid->unrefNoDelete();
        return pGrid;
    }

    SoSeparator* CoinVisualizationFactory::CreatePolygonVisualization(const std::vector<Eigen::Vector3f>& points, VisualizationFactory::Color colorInner, VisualizationFactory::Color colorLine, float lineSize)
    {
        VisualizationFactory::PhongMaterial mat;
        mat.diffuse = colorInner;
        mat.ambient = colorInner;
        mat.transparency = colorInner.transparency;

        return CreatePolygonVisualization(points, mat, colorLine, lineSize);
    }

    SoSeparator* CoinVisualizationFactory::CreatePolygonVisualization(const std::vector<Eigen::Vector3f>& points, VisualizationFactory::PhongMaterial mat, VisualizationFactory::Color colorLine, float lineSize)
    {
        SoSeparator* visu = new SoSeparator;

        if (points.size() == 0)
        {
            return visu;
        }

        visu->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        visu->addChild(u);

        SoMaterial* m = new SoMaterial;
        m->diffuseColor.setValue(mat.diffuse.r, mat.diffuse.g, mat.diffuse.b);
        m->ambientColor.setValue(mat.ambient.r, mat.ambient.g, mat.ambient.b);
        m->emissiveColor.setValue(mat.emission.r, mat.emission.g, mat.emission.b);
        //        m->shininess.setValue(mat.shininess, mat.shininess, mat.shininess);
        m->specularColor.setValue(mat.specular.r, mat.specular.g, mat.specular.b);
        m->transparency.setValue(mat.transparency);
        visu->addChild(m);

        SoCoordinate3* coordinate3 = new SoCoordinate3;
        SoCoordinate3* coordinate3b = new SoCoordinate3;

        std::vector<SbVec3f> pt;

        for (size_t i = 0; i < points.size(); i++)
        {
            SbVec3f pt(points[i](0), points[i](1), points[i](2));
            coordinate3->point.set1Value(i, pt);
            coordinate3b->point.set1Value(i, pt);
        }

        SbVec3f pt0(points[0](0), points[0](1), points[0](2));
        coordinate3b->point.set1Value(points.size(), pt0);
        visu->addChild(coordinate3);
        SoFaceSet* faceSet = new SoFaceSet;
        faceSet->numVertices.set1Value(0, points.size());
        visu->addChild(faceSet);

        // create line around polygon
        if (lineSize > 0.f && !colorLine.isNone())
        {
            SoSeparator* lineSep = new SoSeparator;
            visu->addChild(lineSep);
            SoMaterial* m2 = new SoMaterial;
            m2->diffuseColor.setValue(colorLine.r, colorLine.g, colorLine.b);
            m2->ambientColor.setValue(colorLine.r, colorLine.g, colorLine.b);
            m2->transparency.setValue(colorLine.transparency);
            lineSep->addChild(m2);
            lineSep->addChild(coordinate3b);

            SoDrawStyle* lineSolutionStyle = new SoDrawStyle();
            lineSolutionStyle->lineWidth.setValue(lineSize);
            lineSep->addChild(lineSolutionStyle);

            SoLineSet* lineSet = new SoLineSet;
            lineSet->numVertices.set1Value(0, points.size() + 1);
            //faceSet->startIndex.setValue(0);
            lineSep->addChild(lineSet);
        }

        visu->unrefNoDelete();
        return visu;
    }


    SoSeparator* CoinVisualizationFactory::CreateConvexHull2DVisualization(const MathTools::ConvexHull2DPtr ch, MathTools::Plane& p, VisualizationFactory::Color colorInner /*= VisualizationFactory::Color::Blue()*/, VisualizationFactory::Color colorLine /*= VisualizationFactory::Color::Black()*/, float lineSize /*= 5.0f*/, const Eigen::Vector3f& offset /*=Eigen::Vector3f::Zero() */)
    {
        if (!ch)
        {
            return new SoSeparator;
        }

        std::vector<Eigen::Vector3f> cvHull3d;

        for (size_t u = 0; u < ch->vertices.size(); u++)
        {
            Eigen::Vector3f pt3d = MathTools::planePoint3D(ch->vertices[u], p);
            pt3d += offset;
            cvHull3d.push_back(pt3d);
        }

        return CoinVisualizationFactory::CreatePolygonVisualization(cvHull3d, colorInner, colorLine, lineSize);
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(RobotPtr robot, SceneObject::VisualizationType visuType)
    {
        if (!robot)
        {
            return new SoSeparator;
        }

        boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationRobot = robot->getVisualization<CoinVisualization>(visuType);

        if (visualizationRobot)
        {
            SoSeparator* result = new SoSeparator();
            result->ref();
            result->addChild(visualizationRobot->getCoinVisualization());
            result->unrefNoDelete();
            return result;
        }

        return new SoSeparator;
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(SceneObjectPtr object, SceneObject::VisualizationType visuType)
    {
        if (!object)
        {
            return new SoSeparator;
        }

        boost::shared_ptr<VirtualRobot::CoinVisualization> visualizationObject = object->getVisualization<CoinVisualization>(visuType);

        if (visualizationObject)
        {
            SoSeparator* result = new SoSeparator();
            result->ref();
            result->addChild(visualizationObject->getCoinVisualization());
            result->unrefNoDelete();
            return result;
        }

        return new SoSeparator;

    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(VisualizationNodePtr visu)
    {
        boost::shared_ptr< CoinVisualizationNode > coinVisu(boost::dynamic_pointer_cast< CoinVisualizationNode >(visu));

        if (!coinVisu)
        {
            return new SoSeparator;
        }

        return coinVisu->getCoinVisualization();
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(EndEffector::ContactInfo& contact, float frictionConeHeight,  float frictionConeRadius, bool scaleAccordingToApproachDir)
    {
        SoSeparator* result = new SoSeparator();
        result->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        result->addChild(u);

        // add gfx for contact point on object
        SoSeparator* sep = new SoSeparator();
        SoTranslation* tr = new SoTranslation();
        SoMaterial* mat = new SoMaterial();
        mat->diffuseColor.setValue(1.0f, 0, 0);
        tr->translation.setValue(contact.contactPointObstacleGlobal(0), contact.contactPointObstacleGlobal(1), contact.contactPointObstacleGlobal(2));
        SoSphere* sph = new SoSphere();
        sph->radius.setValue(1);
        sep->addChild(mat);
        sep->addChild(tr);
        sep->addChild(sph);
        result->addChild(sep);

        // add gfx for contact point on finger
        SoSeparator* sep2 = new SoSeparator();
        SoTranslation* tr2 = new SoTranslation();
        tr2->translation.setValue(contact.contactPointFingerGlobal(0), contact.contactPointFingerGlobal(1), contact.contactPointFingerGlobal(2));
        SoSphere* sph2 = new SoSphere();
        sph2->radius.setValue(1);
        sep2->addChild(tr2);
        sep2->addChild(sph2);
        result->addChild(sep2);

        Eigen::Vector3f n;
        n = contact.contactPointObstacleGlobal - contact.contactPointFingerGlobal;

        if (scaleAccordingToApproachDir && n.norm() > 1e-10)
        {
            float factor = n.dot(contact.approachDirectionGlobal) / n.norm();

            frictionConeHeight *= factor;
            frictionConeRadius *= factor;
        }

        // add gfx for approach direction
        SoSeparator* sep3 = new SoSeparator();
        SoMatrixTransform* tr3 = new SoMatrixTransform();
        SbVec3f transl3(contact.contactPointObstacleGlobal(0), contact.contactPointObstacleGlobal(1), contact.contactPointObstacleGlobal(2));
        // compute rotation
        SbVec3f rotFrom(1.0f, 0.0f, 0.0f);
        SbVec3f rotTo;
        rotTo[0] = n[0];
        rotTo[1] = n[1];
        rotTo[2] = n[2];
        SbRotation rot3(rotFrom, rotTo);

        SbVec3f sc3;
        sc3[0] = sc3[1] = sc3[2] = 1.0f;
        SbMatrix m3;
        m3.setTransform(transl3, rot3, sc3);
        tr3->matrix.setValue(m3);

        // create cone
        float fConeHeight = frictionConeHeight;
        float fConeRadius = frictionConeRadius;
        SoCone* cone3 = new SoCone();
        cone3->bottomRadius = fConeRadius;
        cone3->height = fConeHeight;
        SoSeparator* ConeSep = new SoSeparator;
        SbMatrix orientCone;
        SbMatrix orientCone2;
        orientCone.makeIdentity();
        SbVec3f orientConeA(0.0f, 0.0f, 1.0f);
        SbRotation orientConeR(orientConeA, (float)(-M_PI / 2.0f));
        orientCone.setRotate(orientConeR);
        SbVec3f coneTr(0, -fConeHeight / 2.0f, 0);
        orientCone2.setTranslate(coneTr);
        SoMatrixTransform* coneOri = new SoMatrixTransform();
        coneOri->matrix.setValue(orientCone2.multRight(orientCone));
        ConeSep->addChild(coneOri);
        ConeSep->addChild(cone3);
        // material
        SoMaterial* mat3 = new SoMaterial();
        mat3->diffuseColor.setValue(0.2f, 0.7f, 0.2f);
        mat3->ambientColor.setValue(0.2f, 0.7f, 0.2f);
        mat3->transparency.setValue(0.5f);

        sep3->addChild(mat3);
        sep3->addChild(tr3);
        sep3->addChild(ConeSep);
        result->addChild(sep3);

        result->unrefNoDelete();
        return result;
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(EndEffector::ContactInfoVector& contacts, float frictionConeHeight,  float frictionConeRadius, bool scaleAccordingToApproachDir)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();

        for (size_t i = 0; i < contacts.size(); i++)
        {
            res->addChild(getCoinVisualization(contacts[i], frictionConeHeight, frictionConeRadius, scaleAccordingToApproachDir));
        }

        res->unrefNoDelete();
        return res;
    }

    //#define SHOW_TRIMESH_NORMALS
    SoNode* CoinVisualizationFactory::getCoinVisualization(TriMeshModelPtr model)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        if (!model)
        {
            return res;
        }

        bool materialMode = false;
        bool colorMode = false;

        if (model->materials.size() > 0)
        {
            materialMode = true;

            SoMaterialBinding* myBinding = new SoMaterialBinding;
            myBinding->value = SoMaterialBinding::PER_FACE_INDEXED;
            res->addChild(myBinding);

            SbColor* matAmb = new SbColor[model->materials.size()];
            SbColor* matDif = new SbColor[model->materials.size()];
            SbColor* matSpec = new SbColor[model->materials.size()];
            float* transp = new float[model->materials.size()];
            float* shin = new float[model->materials.size()];

            for (size_t i = 0; i < model->materials.size(); i++)
            {
                matAmb[i].setValue(model->materials[i].ambient.r, model->materials[i].ambient.g, model->materials[i].ambient.b);
                matDif[i].setValue(model->materials[i].diffuse.r, model->materials[i].diffuse.g, model->materials[i].diffuse.b);
                matSpec[i].setValue(model->materials[i].specular.r, model->materials[i].specular.g, model->materials[i].specular.b);
                transp[i] = model->materials[i].transparency;
                shin[i] = model->materials[i].shininess;
            }


            // Define colors for the faces
            SoMaterial* myMaterials = new SoMaterial;
            myMaterials->diffuseColor.setValues(0, model->materials.size(), matDif);
            myMaterials->specularColor.setValues(0, model->materials.size(), matSpec);
            myMaterials->ambientColor.setValues(0, model->materials.size(), matAmb);
            myMaterials->transparency.setValues(0, model->materials.size(), transp);
            myMaterials->shininess.setValues(0, model->materials.size(), shin);
            res->addChild(myMaterials);

            delete[] matAmb;
            delete[] matDif;
            delete[] matSpec;
            delete[] shin;
            delete[] transp;

        }
        else if (model->colors.size() > 0)
        {
            colorMode = true;

            SoMaterialBinding* myBinding = new SoMaterialBinding;
            myBinding->value = SoMaterialBinding::PER_VERTEX_INDEXED;
            res->addChild(myBinding);

            SbColor* matAmb = new SbColor[model->colors.size()];
            SbColor* matDif = new SbColor[model->colors.size()];
            float* transp = new float[model->colors.size()];

            for (size_t i = 0; i < model->colors.size(); i++)
            {
                matAmb[i].setValue(model->colors[i].r, model->colors[i].g, model->colors[i].b);
                matDif[i].setValue(model->colors[i].r, model->colors[i].g, model->colors[i].b);
                transp[i] = model->colors[i].transparency;
            }

            // Define colors for the faces
            SoMaterial* myMaterials = new SoMaterial;
            myMaterials->diffuseColor.setValues(0, model->materials.size(), matDif);
            myMaterials->ambientColor.setValues(0, model->materials.size(), matAmb);
            myMaterials->transparency.setValues(0, model->materials.size(), transp);
            res->addChild(myMaterials);
            delete[] matAmb;
            delete[] matDif;
            delete[] transp;
        }

        // define vertex array
        SbVec3f* vertexPositions = new SbVec3f[model->vertices.size()];

        for (size_t i = 0; i < model->vertices.size(); i++)
        {
            vertexPositions[i].setValue(model->vertices[i](0), model->vertices[i](1), model->vertices[i](2));
        }

        // Define coordinates for vertices
        SoCoordinate3* myCoords = new SoCoordinate3;
        myCoords->point.setValues(0, model->vertices.size(), vertexPositions);
        res->addChild(myCoords);
        delete[] vertexPositions;

        // define nomals array
        if (model->normals.size() > 0)
        {
            // per vertex normals
            SbVec3f* normalsArray = new SbVec3f[model->normals.size()];

            for (size_t i = 0; i < model->normals.size(); i++)
            {
                normalsArray[i].setValue(model->normals[i](0), model->normals[i](1), model->normals[i](2));
            }

            // Define coordinates for vertices
            SoNormal* normals = new SoNormal;
            normals->vector.setValues(0, model->normals.size(), normalsArray);
            res->addChild(normals);
            delete[] normalsArray;
        }
        else
        {
            // per face normals
            SbVec3f* normalsArray = new SbVec3f[model->faces.size()];

            for (size_t i = 0; i < model->faces.size(); i++)
            {
                normalsArray[i].setValue(model->faces[i].normal(0), model->faces[i].normal(1), model->faces[i].normal(2));
            }

            // Define coordinates for vertices
            SoNormal* normals = new SoNormal;
            normals->vector.setValues(0, model->faces.size(), normalsArray);
            res->addChild(normals);
            delete[] normalsArray;
        }

        SoNormalBinding* normBinding = new SoNormalBinding;
        normBinding->value = SoNormalBinding::PER_VERTEX_INDEXED;
        res->addChild(normBinding);


        // define faces and normals
        int32_t* faces = new int32_t[model->faces.size() * 4];
        int32_t* normalIndx = new int32_t[model->faces.size() * 4];

        for (size_t i = 0; i < model->faces.size(); i++)
        {
            faces[i * 4] = model->faces[i].id1;
            faces[i * 4 + 1] = model->faces[i].id2;
            faces[i * 4 + 2] = model->faces[i].id3;
            faces[i * 4 + 3] = SO_END_FACE_INDEX;

            if (model->normals.size() > 0)
            {
                normalIndx[i * 4] = model->faces[i].idNormal1;
                normalIndx[i * 4 + 1] = model->faces[i].idNormal2;
                normalIndx[i * 4 + 2] = model->faces[i].idNormal3;
                normalIndx[i * 4 + 3] = SO_END_FACE_INDEX;
            }
            else
            {
                normalIndx[i * 4] = i;
                normalIndx[i * 4 + 1] = i;
                normalIndx[i * 4 + 2] = i;
                normalIndx[i * 4 + 3] = SO_END_FACE_INDEX;
            }
        }

        SoIndexedFaceSet* myFaceSet = new SoIndexedFaceSet;

        // add face vector
        myFaceSet->coordIndex.setValues(0, model->faces.size() * 4, faces);

        // add normals vector
        myFaceSet->normalIndex.setValues(0, model->faces.size() * 4, normalIndx);

        res->addChild(myFaceSet);

        delete[] faces;
        delete[] normalIndx;

        if (materialMode)
        {
            int32_t* matInx = new int32_t[model->faces.size()];

            for (size_t i = 0; i < model->faces.size(); i++)
            {
                if (model->faces[i].idMaterial < model->materials.size())
                {
                    matInx[i] = model->faces[i].idMaterial;
                }
                else
                {
                    matInx[i] = 0;
                }
            }

            myFaceSet->materialIndex.setValues(0, model->faces.size(), matInx);
            delete[] matInx;
        }
        else if (colorMode)
        {
            int32_t* matInx = new int32_t[model->faces.size() * 3];

            for (size_t i = 0; i < model->faces.size(); i++)
            {
                if (model->faces[i].idColor1 < model->colors.size())
                {
                    matInx[i * 3] = model->faces[i].idColor1;
                    matInx[i * 3 + 1] = model->faces[i].idColor2;
                    matInx[i * 3 + 2] = model->faces[i].idColor3;
                }
                else
                {
                    matInx[i * 3] = 0;
                    matInx[i * 3 + 1] = 0;
                    matInx[i * 3 + 2] = 0;
                }
            }

            myFaceSet->materialIndex.setValues(0, model->faces.size() * 3, matInx);
            delete[] matInx;
        }

#ifdef SHOW_TRIMESH_NORMALS
        Eigen::Vector3f z(0, 0, 1.0f);
        SoSeparator* arrow = CreateArrow(z, 30.0f, 1.5f);
        arrow->ref();

        if (model->normals.size() > 0)
        {
            for (size_t i = 0; i < model->faces.size(); i++)
            {
                unsigned int id1 = model->faces[i].id1;
                unsigned int id2 = model->faces[i].id2;
                unsigned int id3 = model->faces[i].id3;
                Eigen::Vector3f v1 = model->vertices[id1];
                Eigen::Vector3f v2 = model->vertices[id2];
                Eigen::Vector3f v3 = model->vertices[id3];

                unsigned int normalIndx1 = model->faces[i].idNormal1;
                unsigned int normalIndx2 = model->faces[i].idNormal2;
                unsigned int normalIndx3 = model->faces[i].idNormal3;
                Eigen::Vector3f normal1 = model->normals[normalIndx1];
                Eigen::Vector3f normal2 = model->normals[normalIndx2];
                Eigen::Vector3f normal3 = model->normals[normalIndx3];

                if (fabs(normal1.norm() - 1.0f) > 1.1)
                {
                    VR_ERROR << "Wrong normal, norm:" << normal1.norm() << endl;
                }

                if (fabs(normal2.norm() - 1.0f) > 1.1)
                {
                    VR_ERROR << "Wrong normal, norm:" << normal2.norm() << endl;
                }

                if (fabs(normal3.norm() - 1.0f) > 1.1)
                {
                    VR_ERROR << "Wrong normal, norm:" << normal3.norm() << endl;
                }

                SoMatrixTransform* mt1 = new SoMatrixTransform;
                SoMatrixTransform* mt2 = new SoMatrixTransform;
                SoMatrixTransform* mt3 = new SoMatrixTransform;

                MathTools::Quaternion q1 = MathTools::getRotation(z, normal1);
                MathTools::Quaternion q2 = MathTools::getRotation(z, normal2);
                MathTools::Quaternion q3 = MathTools::getRotation(z, normal3);
                Eigen::Matrix4f mat1 = MathTools::quat2eigen4f(q1);
                Eigen::Matrix4f mat2 = MathTools::quat2eigen4f(q2);
                Eigen::Matrix4f mat3 = MathTools::quat2eigen4f(q3);
                mat1.block(0, 3, 3, 1) = v1;
                mat2.block(0, 3, 3, 1) = v2;
                mat3.block(0, 3, 3, 1) = v3;
                SbMatrix m1(reinterpret_cast<SbMat*>(mat1.data()));
                SbMatrix m2(reinterpret_cast<SbMat*>(mat2.data()));
                SbMatrix m3(reinterpret_cast<SbMat*>(mat3.data()));
                mt1->matrix.setValue(m1);
                mt2->matrix.setValue(m2);
                mt3->matrix.setValue(m3);
                SoSeparator* sn1 = new SoSeparator();
                sn1->addChild(mt1);
                sn1->addChild(arrow);
                res->addChild(sn1);
                SoSeparator* sn2 = new SoSeparator();
                sn2->addChild(mt2);
                sn2->addChild(arrow);
                res->addChild(sn2);
                SoSeparator* sn3 = new SoSeparator();
                sn3->addChild(mt3);
                sn3->addChild(arrow);
                res->addChild(sn3);
            }
        }

        arrow->unref();
#endif
        res->unrefNoDelete();
        return res;

    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(TriMeshModelPtr model, bool showNormals, VisualizationFactory::Color color, bool showLines)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);
        Eigen::Vector3f v1, v2, v3;
        Eigen::Vector3f z(0, 0, 1.0f);
        SoSeparator* arrow = CreateArrow(z, 30.0f, 1.5f);
        arrow->ref();
        float lineSize = 4.0f;
        VisualizationFactory::Color lineColor = VisualizationFactory::Color::Black();

        if (!showLines)
        {
            lineColor = VisualizationFactory::Color::None();
            lineSize = 0.0f;
        }

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
            //            SoSeparator* s = CreatePolygonVisualization(v,color,lineColor,lineSize);

            VisualizationFactory::Color triColor = (model->colors.size() == 0) ? color : model->colors[model->faces[i].idColor1];

            SoSeparator* s;

            if (model->faces[i].idMaterial >= model->materials.size())
            {
                s = CreatePolygonVisualization(v, triColor, lineColor, lineSize);
            }
            else
            {
                VisualizationFactory::PhongMaterial mat = model->materials[model->faces[i].idMaterial];
                s = CreatePolygonVisualization(v, mat, lineColor, lineSize);
            }

            res->addChild(s);

            if (showNormals)
            {
                v1 = (v1 + v2 + v3) / 3.0f;
                SoMatrixTransform* mt = new SoMatrixTransform;

                Eigen::Vector3f normal = model->faces[i].normal;
                MathTools::Quaternion q = MathTools::getRotation(z, normal);
                Eigen::Matrix4f mat = MathTools::quat2eigen4f(q);
                //Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
                mat.block(0, 3, 3, 1) = v1;
                SbMatrix m(reinterpret_cast<SbMat*>(mat.data()));
                mt->matrix.setValue(m);
                SoSeparator* sn = new SoSeparator();
                sn->addChild(mt);
                sn->addChild(arrow);
                res->addChild(sn);
            }

        }

        arrow->unref();
        res->unrefNoDelete();
        return res;
    }




    SoSeparator* CoinVisualizationFactory::CreateBBoxVisualization(const BoundingBox& bbox, bool wireFrame)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        SoDrawStyle* ds = new SoDrawStyle;

        if (wireFrame)
        {
            ds->style = SoDrawStyle::LINES;
        }
        else
        {
            ds->style = SoDrawStyle::FILLED;
        }

        res->addChild(ds);

        SoTranslation* tr = new SoTranslation();
        Eigen::Vector3f mi = bbox.getMin();
        Eigen::Vector3f ma = bbox.getMax();
        float x1 = std::min(mi(0), ma(0));
        float x2 = std::max(mi(0), ma(0));
        float y1 = std::min(mi(1), ma(1));
        float y2 = std::max(mi(1), ma(1));
        float z1 = std::min(mi(2), ma(2));
        float z2 = std::max(mi(2), ma(2));
        float x = x1 + (x2 - x1) * 0.5f;
        float y = y1 + (y2 - y1) * 0.5f;
        float z = z1 + (z2 - z1) * 0.5f;
        tr->translation.setValue(x, y, z);
        res->addChild(tr);

        SoCube* c = new SoCube;
        c->width = x2 - x1;
        c->height = y2 - y1;
        c->depth = z2 - z1;

        res->addChild(c);
        res->unrefNoDelete();
        return res;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createBoundingBox(const BoundingBox& bbox, bool wireFrame)
    {
        SoSeparator* res = CreateBBoxVisualization(bbox, wireFrame);

        VisualizationNodePtr node(new CoinVisualizationNode(res));
        return node;

    }

    SoSeparator* CoinVisualizationFactory::CreateGraspVisualization(GraspPtr grasp, SoSeparator* eefVisu, const Eigen::Matrix4f& pose /*= Eigen::Matrix4f::Identity()*/)
    {
        if (!grasp || !eefVisu)
        {
            return new SoSeparator;
        }

        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::METERS;
        res->addChild(u);

        // grasp and transform
        Eigen::Matrix4f mat = pose * grasp->getTransformation().inverse();

        // grasp visu
        SoSeparator* sepGrasp = new SoSeparator;
        res->addChild(sepGrasp);

        // transform
        SoMatrixTransform* mT = getMatrixTransformScaleMM2M(mat);
        sepGrasp->addChild(mT);

        // eef Visu
        sepGrasp->addChild(eefVisu);

        res->unrefNoDelete();
        return res;
    }

    SoSeparator* CoinVisualizationFactory::CreateGraspVisualization(GraspPtr grasp, EndEffectorPtr eef, const Eigen::Matrix4f& pose /*= Eigen::Matrix4f::Identity()*/, SceneObject::VisualizationType visu)
    {
        THROW_VR_EXCEPTION_IF(!grasp, "NULL data");
        SoSeparator* eefV = CreateEndEffectorVisualization(eef, visu);

        if (!eefV)
        {
            return new SoSeparator();
        }

        eefV->ref();
        SoSeparator* res = CreateGraspVisualization(grasp, eefV, pose);
        eefV->unrefNoDelete();
        return res;
    }

    SoSeparator* CoinVisualizationFactory::CreateGraspSetVisualization(GraspSetPtr graspSet, EndEffectorPtr eef, const Eigen::Matrix4f& pose /*= Eigen::Matrix4f::Identity()*/, SceneObject::VisualizationType visu)
    {
        THROW_VR_EXCEPTION_IF(!graspSet, "NULL data");
        SoSeparator* visual = new SoSeparator;
        visual->ref();
        SoSeparator* eefV = CreateEndEffectorVisualization(eef, visu);
        visual->addChild(eefV);

        SoSeparator* res = new SoSeparator;
        res->ref();

        for (unsigned int i = 0; i < graspSet->getSize(); i++)
        {
            // grasp and transform
            GraspPtr g = graspSet->getGrasp(i);
            SoSeparator* sepGrasp = CreateGraspVisualization(g, visual, pose);
            res->addChild(sepGrasp);
        }

        visual->unrefNoDelete();
        res->unrefNoDelete();
        return res;
    }

    SoSeparator* CoinVisualizationFactory::CreateEndEffectorVisualization(EndEffectorPtr eef, SceneObject::VisualizationType visu)
    {
        //THROW_VR_EXCEPTION_IF (!eef,"NULL data");
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::METERS;
        res->addChild(u);
        RobotNodePtr tcp;
        bool ok = true;

        if (!eef)
        {
            ok = false;
        }
        else
        {
            tcp = eef->getTcp();

            if (!tcp)
            {
                VR_ERROR << " No tcp in eef " << eef->getName() << endl;
                ok = false;
            }
        }

        if (!ok)
        {
            SoSphere* s = new SoSphere();
            s->radius.setValue(0.005f);
            res->addChild(s);
        }
        else
        {
            RobotPtr r = eef->createEefRobot(eef->getName(), eef->getName());
            RobotNodePtr tcpN = r->getEndEffector(eef->getName())->getTcp();
            r->setGlobalPoseForRobotNode(tcpN, Eigen::Matrix4f::Identity());
            res->addChild(CoinVisualizationFactory::getCoinVisualization(r, visu));
        }

        res->unrefNoDelete();
        return res;
    }

    SoSeparator* CoinVisualizationFactory::CreatePointVisualization(const MathTools::ContactPoint& point, bool showNormals /*= false*/)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);
        SoMatrixTransform* mt = new SoMatrixTransform;
        mt->matrix.setValue(getSbMatrixVec(point.p));
        res->addChild(mt);

        SoSphere* s = new SoSphere;
        s->radius = 10.0f;
        res->addChild(s);

        if (showNormals)
        {
            res->addChild(CreateArrow(point.n));
        }

        res->unrefNoDelete();
        return res;
    }

    SoSeparator* CoinVisualizationFactory::CreatePointsVisualization(const std::vector<MathTools::ContactPoint>& points, bool showNormals /*= false*/)
    {
        SoSeparator* res = new SoSeparator;
        std::vector<MathTools::ContactPoint>::const_iterator i = points.begin();

        while (i != points.end())
        {
            res->addChild(CreatePointVisualization(*i, showNormals));
            i++;
        }

        return res;
    }

    SbMatrix CoinVisualizationFactory::getSbMatrix(const Eigen::Matrix4f& m)
    {
        SbMatrix res(reinterpret_cast<const SbMat*>(m.data()));
        return res;
    }

    SbMatrix CoinVisualizationFactory::getSbMatrixVec(const Eigen::Vector3f& p)
    {
        SbMatrix res;
        res.makeIdentity();
        res.setTranslate(SbVec3f(p(0), p(1), p(2)));
        return res;
    }

    SoSeparator* CoinVisualizationFactory::CreateArrow(const Eigen::Vector3f& n, float length, float width, const Color& color)
    {
        float coneHeight = width * 6.0f;
        float coneBotomRadius = width * 2.5f;
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        SbVec3f objNormal(n(0), n(1), n(2));
        SbMatrix objNormalTrafo;
        objNormalTrafo.makeIdentity();
        SbRotation objNormalRot(SbVec3f(0, 1.0f, 0), objNormal);

        // get rif of warnings when angle==0
        SbVec3f axis;
        float angle;
        objNormalRot.getValue(axis, angle);

        if (angle != 0)
        {
            objNormalTrafo.setRotate(objNormalRot);
        }

        SoMatrixTransform* mt = new SoMatrixTransform;
        mt->matrix.setValue(objNormalTrafo);
        res->addChild(mt);

        if (!color.isNone())
        {
            SoMaterial* col = new SoMaterial();
            col->ambientColor.setValue(color.r, color.g, color.b);
            col->diffuseColor.setValue(color.r, color.g, color.b);
            col->transparency.setValue(color.transparency);
            res->addChild(col);
        }


        SoTranslation* tr = new SoTranslation;
        tr->translation.setValue(0, length * 0.5f, 0);
        res->addChild(tr);


        SoCylinder* c = new SoCylinder();
        c->radius = width;
        c->height = length;
        res->addChild(c);

        SoTranslation* transl = new SoTranslation;
        transl->translation.setValue(0, length * 0.5f + coneHeight * 0.5f, 0);
        res->addChild(transl);

        SoCone* cone = new SoCone();
        cone->bottomRadius.setValue(coneBotomRadius);
        cone->height.setValue(coneHeight);
        res->addChild(cone);

        res->unrefNoDelete();

        return res;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createTriMeshModelVisualization(TriMeshModelPtr model, bool showNormals, Eigen::Matrix4f& pose, bool showLines)
    {
        SoSeparator* res = new SoSeparator;
        SoNode* res1 = CoinVisualizationFactory::getCoinVisualization(model, showNormals, VisualizationFactory::Color::Gray(), showLines);
        SoMatrixTransform* mt = getMatrixTransformScaleMM2M(pose);
        res->addChild(mt);
        res->addChild(res1);

        VisualizationNodePtr node(new CoinVisualizationNode(res));
        return node;
    }


    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createTriMeshModelVisualization(TriMeshModelPtr model, Eigen::Matrix4f& pose)
    {
        SoSeparator* res = new SoSeparator;
        SoNode* res1 = CoinVisualizationFactory::getCoinVisualization(model);
        SoMatrixTransform* mt = getMatrixTransformScaleMM2M(pose);
        res->addChild(mt);
        res->addChild(res1);

        VisualizationNodePtr node(new CoinVisualizationNode(res));
        return node;
    }

    SoMatrixTransform* CoinVisualizationFactory::getMatrixTransform(Eigen::Matrix4f& m)
    {
        SoMatrixTransform* mt = new SoMatrixTransform;
        SbMatrix m_(reinterpret_cast<SbMat*>(m.data()));
        mt->matrix.setValue(m_);
        return mt;
    }

    SoMatrixTransform* CoinVisualizationFactory::getMatrixTransformScaleMM2M(Eigen::Matrix4f& m)
    {
        SoMatrixTransform* mt = new SoMatrixTransform;
        SbMatrix m_(reinterpret_cast<SbMat*>(m.data()));
        // mm -> m
        m_[3][0] *= 0.001f;
        m_[3][1] *= 0.001f;
        m_[3][2] *= 0.001f;
        mt->matrix.setValue(m_);
        return mt;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createArrow(const Eigen::Vector3f& n, float length /*= 50.0f*/, float width /*= 2.0f*/, const Color& color /*= Color::Gray()*/)
    {
        SoSeparator* res = CreateArrow(n, length, width, color);

        VisualizationNodePtr node(new CoinVisualizationNode(res));
        return node;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createTrajectory(TrajectoryPtr t, Color colorNode, Color colorLine, float nodeSize, float lineSize)
    {
        SoNode* res = getCoinVisualization(t, colorNode, colorLine, nodeSize, lineSize);

        VisualizationNodePtr node(new CoinVisualizationNode(res));
        return node;
    }

    //#define TEST_SHOW_VOXEL

    SoNode* CoinVisualizationFactory::getCoinVisualization(WorkspaceRepresentationPtr reachSpace, int a, int b, int c, /*const Eigen::Vector3f &positionGlobal,*/ int nrBestEntries, SoSeparator* arrow, const VirtualRobot::ColorMap& cm, bool transformToGlobalPose, unsigned char minValue)
    {

        if (!reachSpace || reachSpace->numVoxels[0] <= 0 || reachSpace->numVoxels[1] <= 0 || reachSpace->numVoxels[2] <= 0)
        {
            return NULL;
        }

        //float x[6];
        unsigned int v[6];
        Eigen::Matrix4f m;

        v[0] = a;
        v[1] = b;
        v[2] = c;
        v[3] = 0;
        v[4] = 0;
        v[5] = 0;
        Eigen::Vector3f sizePos;//voxelOrientationLocal, size;
        sizePos(0) = reachSpace->spaceSize[0] / reachSpace->numVoxels[0];
        sizePos(1) = reachSpace->spaceSize[1] / reachSpace->numVoxels[1];
        sizePos(2) = reachSpace->spaceSize[2] / reachSpace->numVoxels[2];
        Eigen::Vector3f posLocal(reachSpace->minBounds[0] + ((float)a + 0.5f)*sizePos(0), reachSpace->minBounds[1] + ((float)b + 0.5f)*sizePos(1), reachSpace->minBounds[2] + ((float)c + 0.5f)*sizePos(2));
#ifdef TEST_SHOW_VOXEL
        Eigen::Vector3f posLocalMin(reachSpace->minBounds[0] + ((float)a)*sizePos(0), reachSpace->minBounds[1] + ((float)b)*sizePos(1), reachSpace->minBounds[2] + ((float)c)*sizePos(2));
        Eigen::Vector3f posLocalMax(reachSpace->minBounds[0] + ((float)a + 1.0f)*sizePos(0), reachSpace->minBounds[1] + ((float)b + 1.0f)*sizePos(1), reachSpace->minBounds[2] + ((float)c + 1.0f)*sizePos(2));

#endif

        Eigen::Vector3f size;//voxelOrientationLocal, size;
        size(0) = reachSpace->spaceSize[3] / reachSpace->numVoxels[3];
        size(1) = reachSpace->spaceSize[4] / reachSpace->numVoxels[4];
        size(2) = reachSpace->spaceSize[5] / reachSpace->numVoxels[5];
        std::map< unsigned char, std::vector<Eigen::Vector3f> > entryRot;

        for (unsigned int d = 0; d < (unsigned int)reachSpace->numVoxels[3]; d++)
        {
            //voxelOrientationLocal(0) = reachSpace->minBounds[3] + (d + 0.5f)*size(0);
            v[3] = d;// reachSpace->minBounds[3] + (d + 0.5f)*size(0);

            for (unsigned int e = 0; e < (unsigned int)reachSpace->numVoxels[4]; e++)
            {
                //voxelOrientationLocal(1) = reachSpace->minBounds[4] + (e + 0.5f)*size(1);
                v[4] = e;//reachSpace->minBounds[4] + (e + 0.5f)*size(1);

                for (unsigned int f = 0; f < (unsigned int)reachSpace->numVoxels[5]; f++)
                {
                    v[5] = f;//reachSpace->minBounds[5] + (f + 0.5f)*size(2);
                    unsigned int entry = reachSpace->data->get(v);

                    if (entry > 0)
                    {
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d + 0.5f)*size(0), reachSpace->minBounds[4] + ((float)e + 0.5f)*size(1), reachSpace->minBounds[5] + ((float)f + 0.5f)*size(2)));
#ifdef TEST_SHOW_VOXEL
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d)*size(0), reachSpace->minBounds[4] + ((float)e)*size(1), reachSpace->minBounds[5] + ((float)f)*size(2)));
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d + 1.0f)*size(0), reachSpace->minBounds[4] + ((float)e)*size(1), reachSpace->minBounds[5] + ((float)f)*size(2)));
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d)*size(0), reachSpace->minBounds[4] + ((float)e + 1.0f)*size(1), reachSpace->minBounds[5] + ((float)f)*size(2)));
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d + 1.0f)*size(0), reachSpace->minBounds[4] + ((float)e + 1.0f)*size(1), reachSpace->minBounds[5] + ((float)f)*size(2)));
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d)*size(0), reachSpace->minBounds[4] + ((float)e)*size(1), reachSpace->minBounds[5] + ((float)f + 1.0f)*size(2)));
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d + 1.0f)*size(0), reachSpace->minBounds[4] + ((float)e)*size(1), reachSpace->minBounds[5] + ((float)f + 1.0f)*size(2)));
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d)*size(0), reachSpace->minBounds[4] + ((float)e + 1.0f)*size(1), reachSpace->minBounds[5] + ((float)f + 1.0f)*size(2)));
                        entryRot[entry].push_back(Eigen::Vector3f(reachSpace->minBounds[3] + ((float)d + 1.0f)*size(0), reachSpace->minBounds[4] + ((float)e + 1.0f)*size(1), reachSpace->minBounds[5] + ((float)f + 1.0f)*size(2)));
#endif
                    }
                }
            }
        }

        if (entryRot.size() == 0)
        {
            return NULL;
        }

        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::METERS;
        res->addChild(u);
        VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
        std::map< unsigned char, std::vector<Eigen::Vector3f> >::reverse_iterator  i = entryRot.rbegin();
        int nr = 0;
        float x[6];

        while (i != entryRot.rend() && nr < nrBestEntries)
        {
            for (size_t j = 0; j < i->second.size(); j++)
            {
                // create visu
                SoSeparator* sep = new SoSeparator;
                float intensity = (float)i->first;

                if (reachSpace->getMaxEntry() > 0)
                {
                    intensity /= (float)reachSpace->getMaxEntry();
                }

                if (intensity > 1.0f)
                {
                    intensity = 1.0f;
                }

                color = cm.getColor(intensity);

                SoMaterial* col = new SoMaterial();
                col->ambientColor.setValue(color.r, color.g, color.b);
                col->diffuseColor.setValue(color.r, color.g, color.b);
                col->transparency.setValue(color.transparency);
                sep->addChild(col);
                Eigen::Matrix4f pose;
                x[0] = posLocal(0);
                x[1] = posLocal(1);
                x[2] = posLocal(2);
                x[3] = i->second[j](0);
                x[4] = i->second[j](1);
                x[5] = i->second[j](2);
                reachSpace->vector2Matrix(x, pose);
                //MathTools::posrpy2eigen4f(posLocal,i->second[j],pose);

                if (transformToGlobalPose)
                {
                    reachSpace->toGlobal(pose);
                }

                SoMatrixTransform* mt = getMatrixTransformScaleMM2M(pose);
                sep->addChild(mt);
                sep->addChild(arrow);
                res->addChild(sep);
                nr++;

                if (nr >= nrBestEntries)
                {
                    break;
                }
            }

            i++;
        }

        res->unrefNoDelete();
        return res;
    }

    SoNode *CoinVisualizationFactory::getCoinVisualization(TSRConstraintPtr constraint, const Color &color)
    {
        SoSeparator *res = new SoSeparator;
        res->ref();

        SoUnits *u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        SoMaterial *m = new SoMaterial;
        m->diffuseColor.setValue(color.r, color.g, color.b);
        m->ambientColor.setValue(color.r, color.g, color.b);
        m->transparency.setValue(color.transparency);
        res->addChild(m);

        SoTransform *t = new SoTransform;
        t->translation.setValue(constraint->getTransformation()(0,3), constraint->getTransformation()(1,3), constraint->getTransformation()(2,3));
        MathTools::Quaternion q = MathTools::eigen4f2quat(constraint->getTransformation());
        t->rotation.setValue(q.x, q.y, q.z, q.w);
        res->addChild(t);

        SoCube *c = new SoCube;
        c->width = fabs(constraint->getBounds()(0,0) - constraint->getBounds()(0,1));
        c->height = fabs(constraint->getBounds()(1,0) - constraint->getBounds()(1,1));
        c->depth = fabs(constraint->getBounds()(2,0) - constraint->getBounds()(2,1));
        res->addChild(c);

        res->unrefNoDelete();
        return res;
    }

    SoNode *CoinVisualizationFactory::getCoinVisualization(BalanceConstraintPtr constraint, const Color& color)
    {
        SoSeparator *res = new SoSeparator;
        res->ref();

        SoUnits *u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        SoSeparator *s1 = new SoSeparator;
        res->addChild(s1);

        SoMaterial *m = new SoMaterial;
        m->diffuseColor.setValue(1, 0, 0);
        m->ambientColor.setValue(1, 0, 0);
        s1->addChild(m);

        Eigen::Vector3f com = constraint->getCoM();
        SoTransform *t = new SoTransform;
        t->translation.setValue(com(0), com(1), 0);
        s1->addChild(t);

        SoSphere *s = new SoSphere;
        s->radius = 10;
        s1->addChild(s);

        t = new SoTransform();
        t->translation.setValue(0, 0, com(2));
        s1->addChild(t);
        s1->addChild(s);

        res->addChild(getCoinVisualization(constraint->getSupportPolygon(), color));

        res->unrefNoDelete();
        return res;
    }

    SoNode *CoinVisualizationFactory::getCoinVisualization(PoseConstraintPtr constraint, const Color &color)
    {
        SoSeparator *res = new SoSeparator;
        res->ref();

        SoUnits *u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        SoMaterial *mat = new SoMaterial;
        mat->diffuseColor.setValue(color.r, color.g, color.b);
        mat->ambientColor.setValue(color.r, color.g, color.b);
        mat->transparency.setValue(color.transparency);
        mat->setOverride(true);
        res->addChild(mat);

        SoTransform *t = new SoTransform;
        t->translation.setValue(constraint->getTarget()(0,3), constraint->getTarget()(1,3), constraint->getTarget()(2,3));
        res->addChild(t);

        SoSphere *sphere = new SoSphere;
        sphere->radius = 50;
        res->addChild(sphere);

        res->unrefNoDelete();
        return res;
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(SupportPolygonPtr supportPolygon, const Color& color)
    {
        SoSeparator *res = new SoSeparator;
        res->ref();

        SoUnits *u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        MathTools::ConvexHull2DPtr convexHull = supportPolygon->getSupportPolygon2D();
        MathTools::Plane floor = supportPolygon->getFloorPlane();

        if(convexHull)
        {
            SoMaterial *mat = new SoMaterial;
            mat->diffuseColor.setValue(color.r, color.g, color.b);
            mat->ambientColor.setValue(color.r, color.g, color.b);
            res->addChild(mat);

            SoDrawStyle *d = new SoDrawStyle;
            d->lineWidth.setValue(3);
            res->addChild(d);

            SoCoordinate3 *coordinate = new SoCoordinate3;
            for (size_t i = 0; i < convexHull->segments.size(); i++)
            {
                int i1 = convexHull->segments[i].id1;
                int i2 = convexHull->segments[i].id2;

                if(i == 0)
                {
                    coordinate->point.set1Value(i, convexHull->vertices[i1].x(), convexHull->vertices[i1].y(), floor.p.z());
                }

                coordinate->point.set1Value(i+1, convexHull->vertices[i2].x(), convexHull->vertices[i2].y(), floor.p.z());
            }
            res->addChild(coordinate);

            SoLineSet *lineSet = new SoLineSet;
            res->addChild(lineSet);

            SoSeparator *s2 = new SoSeparator;
            res->addChild(s2);

            Eigen::Vector2f center = MathTools::getConvexHullCenter(convexHull);
            SoTransform *t = new SoTransform;
            t->translation.setValue(center.x(), center.y(), floor.p.z());
            res->addChild(t);

            SoSphere *s = new SoSphere;
            s->radius = 10;
            res->addChild(s);
        }

        res->unrefNoDelete();
        return res;
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(WorkspaceRepresentationPtr reachSpace, const Eigen::Vector3f& fixedEEFOrientationGlobalRPY, VirtualRobot::ColorMap cm, bool transformToGlobalPose, const Eigen::Vector3f& axis, unsigned char minValue, float arrowSize)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();

        if (!reachSpace || reachSpace->numVoxels[0] <= 0 || reachSpace->numVoxels[1] <= 0 || reachSpace->numVoxels[2] <= 0)
        {
            res->unrefNoDelete();
            return res;
        }

        SoUnits* u = new SoUnits();
        u->units = SoUnits::METERS;
        res->addChild(u);
        Eigen::Matrix4f m;
        Eigen::Vector3f oLocal = fixedEEFOrientationGlobalRPY;
        MathTools::rpy2eigen4f(fixedEEFOrientationGlobalRPY(0), fixedEEFOrientationGlobalRPY(1), fixedEEFOrientationGlobalRPY(2), m);
        reachSpace->toLocal(m);
        float x[6];
        reachSpace->matrix2Vector(m, x);
        oLocal(0) = x[3];
        oLocal(1) = x[4];
        oLocal(2) = x[5];
        /*if (reachSpace->baseNode)
        {
        m = reachSpace->baseNode->toLocalCoordinateSystem(m);
        MathTools::eigen4f2rpy(m,oLocal);
        }*/

        Eigen::Vector3f voxelPosition, size;
        int d, e, f;

        size(0) = reachSpace->spaceSize[0] / reachSpace->numVoxels[0];
        size(1) = reachSpace->spaceSize[1] / reachSpace->numVoxels[1];
        size(2) = reachSpace->spaceSize[2] / reachSpace->numVoxels[2];
        float minS = size(0);

        if (size(1) < minS)
        {
            minS = size(1);
        }

        if (size(2) < minS)
        {
            minS = size(2);
        }

        if (arrowSize != 0)
        {
            minS = arrowSize;
        }

        //Eigen::Vector3f zAxis(0,0,1.0f);
        int value;
        VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
        SoSeparator* arrow = CreateArrow(axis, minS * 0.75f, minS / 20.0f, color);

        if (minValue <= 0)
        {
            minValue = 1;
        }

        for (int a = 0; a < reachSpace->numVoxels[0]; a++)
        {
            voxelPosition(0) = reachSpace->minBounds[0] + (a + 0.5f) * size(0);

            for (int b = 0; b < reachSpace->numVoxels[1]; b++)
            {
                voxelPosition(1) = reachSpace->minBounds[1] + (b + 0.5f) * size(1);

                //int cSize = testMode? numVoxels[2]/2 : numVoxels[2];
                int cSize = reachSpace->numVoxels[2];

                for (int c = 0; c < cSize; c++)
                {
                    voxelPosition(2) = reachSpace->minBounds[2] + (c + 0.5f) * size(2);

                    // get voxel from orientation
                    d = (int)(((oLocal(0) - reachSpace->minBounds[3]) / reachSpace->spaceSize[3]) * (float)reachSpace->numVoxels[3]);
                    e = (int)(((oLocal(1) - reachSpace->minBounds[4]) / reachSpace->spaceSize[4]) * (float)reachSpace->numVoxels[4]);
                    f = (int)(((oLocal(2) - reachSpace->minBounds[5]) / reachSpace->spaceSize[5]) * (float)reachSpace->numVoxels[5]);

                    if (d >= 0 && d <= reachSpace->numVoxels[3] && e >= 0 && e <= reachSpace->numVoxels[4] && f >= 0 && f <= reachSpace->numVoxels[5])
                    {
                        value = reachSpace->data->get(a, b, c, d, e, f);
                    }
                    else
                    {
                        value = 0;
                    }

                    if (value >= minValue)
                    {
                        reachSpace->vector2Matrix(voxelPosition, oLocal, m);

                        //MathTools::posrpy2eigen4f(voxelPosition,oLocal,m);
                        if (transformToGlobalPose)
                        {
                            reachSpace->toGlobal(m);
                            //m = reachSpace->baseNode->toGlobalCoordinateSystem(m);
                        }

                        SoSeparator* sep = new SoSeparator;
                        float intensity = (float)value;

                        if (reachSpace->getMaxEntry() > 0)
                        {
                            intensity /= (float)reachSpace->getMaxEntry();
                        }

                        if (intensity > 1.0f)
                        {
                            intensity = 1.0f;
                        }

                        color = cm.getColor(intensity);

                        SoMaterial* col = new SoMaterial();
                        col->ambientColor.setValue(color.r, color.g, color.b);
                        col->diffuseColor.setValue(color.r, color.g, color.b);
                        col->transparency.setValue(color.transparency);
                        sep->addChild(col);
                        SoMatrixTransform* mt = getMatrixTransformScaleMM2M(m);
                        sep->addChild(mt);
                        sep->addChild(arrow);
                        res->addChild(sep);
                    }
                }
            }
        }

        res->unrefNoDelete();
        return res;
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(WorkspaceRepresentationPtr reachSpace, VirtualRobot::ColorMap cm, const Eigen::Vector3f& axis, bool transformToGlobalPose, unsigned char minValue, float arrowSize, int nrRotations)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();

        if (!reachSpace || reachSpace->numVoxels[0] <= 0 || reachSpace->numVoxels[1] <= 0 || reachSpace->numVoxels[2] <= 0)
        {
            res->unrefNoDelete();
            return res;
        }

        Eigen::Vector3f size;
        size(0) = reachSpace->spaceSize[0] / reachSpace->numVoxels[0];
        size(1) = reachSpace->spaceSize[1] / reachSpace->numVoxels[1];
        size(2) = reachSpace->spaceSize[2] / reachSpace->numVoxels[2];
        float minS = size(0);

        if (size(1) < minS)
        {
            minS = size(1);
        }

        if (size(2) < minS)
        {
            minS = size(2);
        }

        if (arrowSize != 0)
        {
            minS = arrowSize;
        }

        VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
        SoSeparator* arrow = CreateArrow(axis, minS * 0.7f, minS / 25.0f, color);

        Eigen::Vector3f voxelPosition;
        int step = 1;

        for (int a = 0; a < reachSpace->numVoxels[0]; a += step)
        {
            voxelPosition(0) = reachSpace->minBounds[0] + (a + 0.5f) * size(0);

            for (int b = 0; b < reachSpace->numVoxels[1]; b += step)
            {
                voxelPosition(1) = reachSpace->minBounds[1] + (b + 0.5f) * size(1);

                //int cSize = testMode? numVoxels[2]/2 : numVoxels[2];
                int cSize = reachSpace->numVoxels[2];

                for (int c = 0; c < cSize; c += step)
                {
                    voxelPosition(2) = reachSpace->minBounds[2] + (c + 0.5f) * size(2);

                    if (reachSpace->hasEntry(a, b, c))
                    {
                        SoNode* n = getCoinVisualization(reachSpace, a, b, c, nrRotations, arrow, cm, transformToGlobalPose, minValue);

                        if (n)
                        {
                            res->addChild(n);
                        }
                    }
                }
            }
        }

        res->unrefNoDelete();
        return res;
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(WorkspaceRepresentationPtr reachSpace, const VirtualRobot::ColorMap cm, bool transformToGlobalPose)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();

        if (!reachSpace || reachSpace->numVoxels[0] <= 0 || reachSpace->numVoxels[1] <= 0 || reachSpace->numVoxels[2] <= 0)
        {
            res->unrefNoDelete();
            return res;
        }

        Eigen::Vector3f size;
        size(0) = reachSpace->spaceSize[0] / reachSpace->numVoxels[0];
        size(1) = reachSpace->spaceSize[1] / reachSpace->numVoxels[1];
        size(2) = reachSpace->spaceSize[2] / reachSpace->numVoxels[2];
        float minS = size(0);

        if (size(1) < minS)
        {
            minS = size(1);
        }

        if (size(2) < minS)
        {
            minS = size(2);
        }

        VirtualRobot::VisualizationFactory::Color color = VirtualRobot::VisualizationFactory::Color::None();
        float radius = minS * 0.5f * 0.75f;
        Eigen::Vector3f voxelPosition;
        int step = 1;
        int maxValue = 0;

        for (int a = 0; a < reachSpace->numVoxels[0]; a += step)
        {
            for (int b = 0; b < reachSpace->numVoxels[1]; b += step)
            {
                for (int c = 0; c < reachSpace->numVoxels[2]; c += step)
                {
                    int value = reachSpace->sumAngleReachabilities(a, b, c);

                    if (value >= maxValue)
                    {
                        maxValue = value;
                    }
                }
            }
        }

        Eigen::Vector3f resPos;

        for (int a = 0; a < reachSpace->numVoxels[0]; a += step)
        {
            voxelPosition(0) = reachSpace->minBounds[0] + (a + 0.5f) * size(0);

            for (int b = 0; b < reachSpace->numVoxels[1]; b += step)
            {
                voxelPosition(1) = reachSpace->minBounds[1] + (b + 0.5f) * size(1);

                //int cSize = testMode? numVoxels[2]/2 : numVoxels[2];
                int cSize = reachSpace->numVoxels[2];

                for (int c = 0; c < cSize; c += step)
                {
                    voxelPosition(2) = reachSpace->minBounds[2] + (c + 0.5f) * size(2);

                    int value = reachSpace->sumAngleReachabilities(a, b, c);

                    if (value > 0)
                    {
                        resPos = voxelPosition;

                        if (transformToGlobalPose) // && reachSpace->baseNode)
                        {
                            reachSpace->toGlobalVec(resPos);
                            //voxelPosition = reachSpace->baseNode->toGlobalCoordinateSystemVec(voxelPosition);
                        }

                        float intensity = (float)value;

                        if (maxValue > 0)
                        {
                            intensity /= maxValue;
                        }

                        if (intensity > 1.0f)
                        {
                            intensity = 1.0f;
                        }

                        color = cm.getColor(intensity);

                        SoNode* n = CreateVertexVisualization(resPos, radius, color.transparency, color.r, color.g, color.b);

                        if (n)
                        {
                            res->addChild(n);
                        }
                    }
                }
            }
        }

        res->unrefNoDelete();
        return res;
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(TrajectoryPtr t, Color colorNode, Color colorLine, float nodeSize, float lineSize)
    {
        SoSeparator* res = new SoSeparator;

        if (!t)
        {
            return res;
        }

        res->ref();
        RobotNodeSetPtr rns = t->getRobotNodeSet();
        Eigen::VectorXf c;
        rns->getJointValues(c);
        std::vector<Eigen::Matrix4f> ws = t->createWorkspaceTrajectory();

        SoMaterial* materialNodeSolution = new SoMaterial();
        SoMaterial* materialLineSolution = new SoMaterial();
        materialNodeSolution->ambientColor.setValue(colorNode.r, colorNode.g, colorNode.b);
        materialNodeSolution->diffuseColor.setValue(colorNode.r, colorNode.g, colorNode.b);
        materialLineSolution->ambientColor.setValue(colorLine.r, colorLine.g, colorLine.b);
        materialLineSolution->diffuseColor.setValue(colorLine.r, colorLine.g, colorLine.b);
        SoSphere* sphereNodeSolution = new SoSphere();
        sphereNodeSolution->radius.setValue(nodeSize);
        SoDrawStyle* lineSolutionStyle = new SoDrawStyle();
        lineSolutionStyle->lineWidth.setValue(lineSize);

        Eigen::VectorXf actConfig;
        Eigen::VectorXf parentConfig;
        float x, y, z;
        float x2 = 0.0f, y2 = 0.0f, z2 = 0.0f;

        SoSeparator* sep = new SoSeparator();
        res->addChild(sep);
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        sep->addChild(u);


        SoComplexity* comple;
        comple = new SoComplexity();
        comple->value = 1.0f;
        sep->addChild(comple);


        for (size_t i = 0; i < ws.size(); i++)
        {
            // create 3D model for nodes
            SoSeparator* s = new SoSeparator();
            s->addChild(materialNodeSolution);
            SoTranslation* t = new SoTranslation();

            Eigen::Matrix4f m;
            m = ws[i];
            x = m(0, 3);
            y = m(1, 3);
            z = m(2, 3);

            t->translation.setValue(x, y, z);
            s->addChild(t);
            // display a solution node different
            s->addChild(sphereNodeSolution);
            sep->addChild(s);

            if (i > 0) // lines for all configurations
            {
                // create line to parent
                SoSeparator* s2 = new SoSeparator();

                SbVec3f points[2];
                points[0].setValue(x2, y2, z2);
                points[1].setValue(x, y, z);

                s2->addChild(lineSolutionStyle);
                s2->addChild(materialLineSolution);

                SoCoordinate3* coordinate3 = new SoCoordinate3;
                coordinate3->point.set1Value(0, points[0]);
                coordinate3->point.set1Value(1, points[1]);
                s2->addChild(coordinate3);

                SoLineSet* lineSet = new SoLineSet;
                lineSet->numVertices.setValue(2);
                lineSet->startIndex.setValue(0);
                s2->addChild(lineSet);

                sep->addChild(s2);
            }

            x2 = x;
            y2 = y;
            z2 = z;
        } // for

        rns->getRobot()->setJointValues(rns, c);

        res->unrefNoDelete();
        return res;
    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(WorkspaceGridPtr reachGrid, VirtualRobot::ColorMap cm, bool transformToGlobalPose /*= true*/)
    {
        SoSeparator* res = new SoSeparator;

        if (!reachGrid)
        {
            return res;
        }

        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        float minX, maxX, minY, maxY;
        reachGrid->getExtends(minX, maxX, minY, maxY);
        gp(0, 3) = minX;
        gp(1, 3) = minY;

        int nX, nY;

        reachGrid->getCells(nX, nY);

        float sizeX = (maxX - minX) / (float)nX;
        float sizeY = (maxY - minY) / (float)nY;


        float ro, gr, bl;
        SoCube* cube = new SoCube();
        cube->width = sizeX;
        cube->height = sizeY;
        cube->depth = 1.0f;
        int maxEntry = reachGrid->getMaxEntry();

        if (maxEntry == 0)
        {
            maxEntry = 1;
        }



        SoDrawStyle* ds = new SoDrawStyle;
        ds->style = SoDrawStyle::LINES;

        // back-face culling
        SoShapeHints* shapeHints = new SoShapeHints;
        //shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
        shapeHints->vertexOrdering = SoShapeHints::UNKNOWN_ORDERING;
        shapeHints->shapeType = SoShapeHints::SOLID;

        SoBaseColor* bc = new SoBaseColor;
        bc->rgb.setValue(0, 0, 0);

        // keep a solid color
        SoLightModel* lightModel = new SoLightModel;
        lightModel->model = SoLightModel::BASE_COLOR;

        for (int x = 0; x < nX; x++)
        {
            float xPos = minX + (float)x * sizeX + 0.5f * sizeX; // center of voxel

            for (int y = 0; y < nY; y++)
            {
                int v;
                std::vector<GraspPtr> grasps;
                bool ok = reachGrid->getCellEntry(x, y, v, grasps);

                if (ok && v > 0)
                {
                    float yPos = minY + (float)y * sizeY + 0.5f * sizeY; // center of voxel
                    gp(0, 3) = xPos;
                    gp(1, 3) = yPos;

                    SoSeparator* sep1 = new SoSeparator();
                    SoMatrixTransform* matTr = getMatrixTransform(gp); // we are already in mm, no conversion to m needed

                    float intensity = (float)v;
                    intensity /= maxEntry;

                    if (intensity > 1.0f)
                    {
                        intensity = 1.0f;
                    }

                    VirtualRobot::VisualizationFactory::Color color = cm.getColor(intensity);

                    SoMaterial* mat = new SoMaterial();

                    ro = color.r;
                    gr = color.g;
                    bl = color.b;

                    mat->diffuseColor.setValue(ro, gr, bl);
                    mat->ambientColor.setValue(ro, gr, bl);

                    if (intensity > 0)
                    {

                        sep1->addChild(matTr);
                        sep1->addChild(mat);
                        sep1->addChild(cube);

                        SoSeparator* pSepLines = new SoSeparator;
                        sep1->addChild(pSepLines);

                        pSepLines->addChild(ds);
                        pSepLines->addChild(shapeHints);
                        pSepLines->addChild(lightModel);
                        pSepLines->addChild(bc);
                        pSepLines->addChild(cube);

                        res->addChild(sep1);
                    }
                }
            }
        }

        //res->addChild(lines);
        res->unrefNoDelete();
        return res;

    }

    SoNode* CoinVisualizationFactory::getCoinVisualization(WorkspaceRepresentation::WorkspaceCut2DPtr cutXY, VirtualRobot::ColorMap cm, const Eigen::Vector3f& normal)
    {
        SoSeparator* res = new SoSeparator;

        if (!cutXY)
        {
            return res;
        }

        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        //Eigen::Matrix4f gp = cutXY->referenceGlobalPose;
        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        // set z component
        gp(2, 3) = cutXY->referenceGlobalPose(2, 3);

        int nX = cutXY->entries.rows();
        int nY = cutXY->entries.cols();

        float sizeX = (cutXY->maxBounds[0] - cutXY->minBounds[0]) / (float)nX;
        float sizeY = (cutXY->maxBounds[1] - cutXY->minBounds[1]) / (float)nY;


        float ro, gr, bl;
        SoCube* cube = new SoCube();

        if (normal(0) > 0)
        {
            cube->width = sizeX;
            cube->depth = sizeY;
            cube->height = 1.0;
        }
        else if (normal(1) > 0)
        {
            cube->width = 1.0f;
            cube->depth = sizeX;
            cube->height = sizeY;
        }
        else
        {
            cube->width = sizeX;
            cube->depth = 1.0f;
            cube->height = sizeY;
        }

        int maxEntry = cutXY->entries.maxCoeff();

        if (maxEntry == 0)
        {
            maxEntry = 1;
        }



        SoDrawStyle* ds = new SoDrawStyle;
        ds->style = SoDrawStyle::LINES;

        // back-face culling
        SoShapeHints* shapeHints = new SoShapeHints;
        //shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
        shapeHints->vertexOrdering = SoShapeHints::UNKNOWN_ORDERING;
        shapeHints->shapeType = SoShapeHints::SOLID;

        SoBaseColor* bc = new SoBaseColor;
        bc->rgb.setValue(0, 0, 0);

        // keep a solid color
        SoLightModel* lightModel = new SoLightModel;
        lightModel->model = SoLightModel::BASE_COLOR;

        for (int x = 0; x < nX; x++)
        {
            float xPos = cutXY->minBounds[0] + (float)x * sizeX + 0.5f * sizeX; // center of voxel

            for (int y = 0; y < nY; y++)
            {
                int v = cutXY->entries(x, y);

                if (v > 0)
                {
                    float yPos = cutXY->minBounds[1] + (float)y * sizeY + 0.5f * sizeY; // center of voxel
                    gp(0, 3) = xPos;
                    gp(1, 3) = yPos;

                    SoSeparator* sep1 = new SoSeparator();
                    SoMatrixTransform* matTr = getMatrixTransform(gp); // we are in mm unit environment -> no conversion to m needed

                    float intensity = (float)v;
                    intensity /= maxEntry;

                    if (intensity > 1.0f)
                    {
                        intensity = 1.0f;
                    }

                    VirtualRobot::VisualizationFactory::Color color = cm.getColor(intensity);

                    SoMaterial* mat = new SoMaterial();

                    ro = color.r;
                    gr = color.g;
                    bl = color.b;

                    mat->diffuseColor.setValue(ro, gr, bl);
                    mat->ambientColor.setValue(ro, gr, bl);

                    if (intensity > 0)
                    {

                        sep1->addChild(matTr);
                        sep1->addChild(mat);
                        sep1->addChild(cube);

                        SoSeparator* pSepLines = new SoSeparator;
                        sep1->addChild(pSepLines);

                        pSepLines->addChild(ds);
                        pSepLines->addChild(shapeHints);
                        pSepLines->addChild(lightModel);
                        pSepLines->addChild(bc);
                        pSepLines->addChild(cube);

                        res->addChild(sep1);
                    }
                }
            }
        }

        //res->addChild(lines);
        res->unrefNoDelete();
        return res;
    }



    SoSeparator* CoinVisualizationFactory::Create2DMap(const Eigen::MatrixXf& d, float extendCellX, float extendCellY, const VirtualRobot::ColorMap cm, bool drawZeroCells, bool drawLines)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        int nX = d.rows();
        int nY = d.cols();

        VR_ASSERT(nX > 0);
        VR_ASSERT(nY > 0);

        SoTranslation* t = new SoTranslation();
        t->translation.setValue(-extendCellX * nX / 2, -extendCellY * nY / 2, 0);
        res->addChild(t);

        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        float sizeX = extendCellX;
        float sizeY = extendCellY;


        float ro, gr, bl;
        SoCube* cube = new SoCube();
        cube->width = sizeX;
        cube->depth = 1.0;
        cube->height = sizeY;

        float maxEntry = 1.0f;

        if (d.maxCoeff() > 1.0f)
        {
            VR_ERROR << "Maximal coefficient must not be >1!" << endl;
        }

        SoDrawStyle* ds = new SoDrawStyle;
        ds->style = SoDrawStyle::LINES;

        // back-face culling
        SoShapeHints* shapeHints = new SoShapeHints;
        //shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
        shapeHints->vertexOrdering = SoShapeHints::UNKNOWN_ORDERING;
        shapeHints->shapeType = SoShapeHints::SOLID;

        SoBaseColor* bc = new SoBaseColor;
        bc->rgb.setValue(0, 0, 0);

        // keep a solid color
        SoLightModel* lightModel = new SoLightModel;
        lightModel->model = SoLightModel::BASE_COLOR;

        SoSeparator* grid = new SoSeparator;
        res->addChild(lightModel);
        res->addChild(grid);

        for (int x = 0; x < nX; x++)
        {
            float xPos = (float)x * sizeX + 0.5f * sizeX; // center of voxel

            for (int y = 0; y < nY; y++)
            {
                float v = d(x, y);

                if (drawZeroCells || v > 0)
                {
                    float yPos = (float)y * sizeY + 0.5f * sizeY; // center of voxel
                    gp(0, 3) = xPos;
                    gp(1, 3) = yPos;

                    SoSeparator* sep1 = new SoSeparator();
                    SoMatrixTransform* matTr = getMatrixTransform(gp); // we are in mm unit environment -> no conversion to m needed

                    float intensity = (float)v;
                    intensity /= maxEntry;

                    if (intensity > 1.0f)
                    {
                        intensity = 1.0f;
                    }

                    VirtualRobot::VisualizationFactory::Color color = cm.getColor(intensity);

                    SoMaterial* mat = new SoMaterial();

                    ro = color.r;
                    gr = color.g;
                    bl = color.b;

                    mat->diffuseColor.setValue(ro, gr, bl);
                    mat->ambientColor.setValue(ro, gr, bl);

                    if (drawZeroCells || intensity > 0)
                    {

                        sep1->addChild(matTr);
                        sep1->addChild(mat);
                        sep1->addChild(cube);

                        if (drawLines)
                        {
                            SoSeparator* pSepLines = new SoSeparator;
                            sep1->addChild(pSepLines);

                            pSepLines->addChild(ds);
                            pSepLines->addChild(shapeHints);
                            pSepLines->addChild(bc);
                            pSepLines->addChild(cube);
                        }

                        grid->addChild(sep1);
                    }
                }
            }
        }

        //res->addChild(lines);
        res->unrefNoDelete();
        return res;
    }

    SoSeparator* CoinVisualizationFactory::Create2DHeightMap(const Eigen::MatrixXf& d, float extendCellX, float extendCellY, float heightZ, const VirtualRobot::ColorMap cm, bool drawZeroCells, bool drawLines)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        int nX = d.rows();
        int nY = d.cols();

        VR_ASSERT(nX > 0);
        VR_ASSERT(nY > 0);

        SoTranslation* t = new SoTranslation();
        t->translation.setValue(-extendCellX * nX / 2, -extendCellY * nY / 2, 0);
        res->addChild(t);

        Eigen::Matrix4f gp = Eigen::Matrix4f::Identity();
        float sizeX = extendCellX;
        float sizeY = extendCellY;

        /*      SoCube *cube = new SoCube();
                cube->width = sizeX;
                cube->depth = 1.0;
                cube->height = sizeY;*/

        float maxEntry = 1.0f;

        if (d.maxCoeff() > 1.0f)
        {
            VR_ERROR << "Maximal coefficient must not be >1!" << endl;
        }

        SoDrawStyle* ds = new SoDrawStyle;
        ds->style = SoDrawStyle::LINES;

        // back-face culling
        SoShapeHints* shapeHints = new SoShapeHints;
        //shapeHints->vertexOrdering = SoShapeHints::COUNTERCLOCKWISE;
        shapeHints->vertexOrdering = SoShapeHints::UNKNOWN_ORDERING;
        shapeHints->shapeType = SoShapeHints::SOLID;

        SoBaseColor* bc = new SoBaseColor;
        bc->rgb.setValue(0, 0, 0);

        // keep a solid color
        SoLightModel* lightModel = new SoLightModel;
        lightModel->model = SoLightModel::BASE_COLOR;

        SoSeparator* grid = new SoSeparator;
        res->addChild(lightModel);
        res->addChild(grid);

        for (int x = 1; x < nX; x++)
        {
            for (int y = 1; y < nY; y++)
            {
                float v1 = d(x - 1, y - 1);
                float v2 = d(x, y - 1);
                float v3 = d(x - 1, y);
                float v4 = d(x, y);
                float xPos1 = (float)(x - 1) * sizeX + 0.5f * sizeX; // center of voxel
                float xPos2 = (float)(x) * sizeX + 0.5f * sizeX; // center of voxel
                float yPos1 = (float)(y - 1) * sizeY + 0.5f * sizeY; // center of voxel
                float yPos2 = (float)y * sizeY + 0.5f * sizeY; // center of voxel
                float v = (v1 + v2 + v3 + v4) / 4.0f;

                float intensity1 = (float)v1 / maxEntry;
                float height1 = intensity1 * heightZ;
                float intensity2 = (float)v2 / maxEntry;
                float height2 = intensity2 * heightZ;
                float intensity3 = (float)v3 / maxEntry;
                float height3 = intensity3 * heightZ;
                float intensity4 = (float)v4 / maxEntry;
                float height4 = intensity4 * heightZ;

                Eigen::Vector3f p1(xPos1, yPos1, height1);
                Eigen::Vector3f p2(xPos2, yPos1, height2);
                Eigen::Vector3f p3(xPos1, yPos2, height3);
                Eigen::Vector3f p4(xPos2, yPos2, height4);
                std::vector<Eigen::Vector3f> pts;
                pts.push_back(p1);
                pts.push_back(p2);
                pts.push_back(p4);
                pts.push_back(p3);

                float intensity = (float)v;
                intensity /= maxEntry;

                if (intensity > 1.0f)
                {
                    intensity = 1.0f;
                }

                if (drawZeroCells || intensity > 0.)
                {
                    float lineWidth = drawLines ? 4.0f : 0.f;
                    SoSeparator* pol = CreatePolygonVisualization(pts, cm.getColor(intensity), VisualizationFactory::Color::Black(), lineWidth);
                    grid->addChild(pol);
                }
            }
        }

        //res->addChild(lines);
        res->unrefNoDelete();
        return res;
    }

    SoSeparator* CoinVisualizationFactory::Colorize(SoNode* model, VisualizationFactory::Color c)
    {
        SoSeparator* result = new SoSeparator;
        SoBaseColor* bc = new SoBaseColor();
        bc->rgb.setValue(c.r, c.g, c.b);
        bc->rgb.setIgnored(FALSE);
        bc->setOverride(TRUE);
        result->addChild(bc);

        if (model)
        {
            result->addChild(model);
        }

        return result;
    }

    SoOffscreenRenderer* CoinVisualizationFactory::createOffscreenRenderer(int width, int height)
    {
        // Set up the offscreen renderer
        SbViewportRegion vpRegion(width, height);
        SoOffscreenRenderer* offscreenRenderer = new SoOffscreenRenderer(vpRegion);
        offscreenRenderer->setComponents(SoOffscreenRenderer::RGB);
        offscreenRenderer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
        return offscreenRenderer;
    }

    bool CoinVisualizationFactory::renderOffscreen(SoOffscreenRenderer* renderer, RobotNodePtr camNode, SoNode* scene, unsigned char** buffer)
    {
        if (!camNode)
        {
            VR_ERROR << "No cam node to render..." << endl;
            return false;
        }

        //we assume mm
        SoPerspectiveCamera* cam = new SoPerspectiveCamera();
        cam->ref();
        // set camera position and orientation
        Eigen::Matrix4f camPose = camNode->getGlobalPose();
        Eigen::Vector3f camPos = MathTools::getTranslation(camPose);
        float sc = 1.0f;//0.001f;
        cam->position.setValue(camPos[0]*sc, camPos[1]*sc, camPos[2]*sc);
        SbRotation align(SbVec3f(1, 0, 0), (float)(M_PI)); // first align from  default direction -z to +z by rotating with 180 degree around x axis
        SbRotation align2(SbVec3f(0, 0, 1), (float)(-M_PI / 2.0)); // align up vector by rotating with -90 degree around z axis
        SbRotation trans(CoinVisualizationFactory::getSbMatrix(camPose)); // get rotation from global pose
        cam->orientation.setValue(align2 * align * trans); // perform total transformation

        // todo: check these values....
        cam->nearDistance.setValue(10.0f);
        cam->farDistance.setValue(100000.0f);

        //cam->nearDistance.setValue(0.0010f);
        //cam->farDistance.setValue(10.0f);

        bool res = renderOffscreen(renderer, cam, scene, buffer);
        cam->unref();
        return res;
    }



    bool CoinVisualizationFactory::renderOffscreen(SoOffscreenRenderer* renderer, SoCamera* cam, SoNode* scene, unsigned char** buffer)
    {
        if (!renderer || !cam || !scene || buffer == NULL)
        {
            return false;
        }

        // we use MM in VirtualRobot
        SoUnits* unit = new SoUnits();
        unit->units = SoUnits::MILLIMETERS;

        // add all to a inventor scene graph
        SoSeparator* root = new SoSeparator();
        root->ref();
        SoDirectionalLight* light = new SoDirectionalLight;
        root->addChild(light);

        // easy light model, no shadows or something
        //SoLightModel *lightModel = new SoLightModel();
        //lightModel->model = SoLightModel::BASE_COLOR;
        //root->addChild(lightModel);

        root->addChild(unit);
        root->addChild(cam);
        root->addChild(scene);


        bool ok = renderer->render(root) == TRUE ? true : false;
        root->unref();

        static bool renderErrorPrinted = false;
        if (!ok)
        {
            if (!renderErrorPrinted)
            {
                VR_ERROR << "Rendering not successful! This error is printed only once." << endl;
                renderErrorPrinted = true;
            }
            return false;
        }

        *buffer = renderer->getBuffer();
        return true;
    }

    VirtualRobot::VisualizationNodePtr CoinVisualizationFactory::createUnitedVisualization(const std::vector<VisualizationNodePtr>& visualizations) const
    {
        if (visualizations.size() == 0)
        {
            return VisualizationNodePtr();
        }

        SoSeparator* s = new SoSeparator;
        s->ref();

        for (size_t i = 0; i < visualizations.size(); i++)
        {
            if (visualizations[i]->getType() == VisualizationFactory::getName())
            {
                //skip empty visus
                continue;
            }

            if (visualizations[i]->getType() != getName())
            {
                VR_ERROR << "Skipping Visualization " << i << ": Is type " << visualizations[i]->getType() << ", but factory is of type " << getName() << endl;
                continue;
            }

            CoinVisualizationNode* cvn = dynamic_cast<CoinVisualizationNode*>(visualizations[i].get());

            if (cvn)
            {
                SoNode* n = cvn->getCoinVisualization();

                if (n)
                {
                    s->addChild(n->copy(FALSE));
                }
            }
            else
            {
                VR_WARNING << "Invalid type casting to CoinVisualizationNode?!" << endl;
            }
        }

        VisualizationNodePtr result(new CoinVisualizationNode(s));
        s->unrefNoDelete();
        return result;
    }

    void CoinVisualizationFactory::cleanup()
    {
        if (SoDB::isInitialized())
        {
            SoDB::finish();
        }
    }

    SoSeparator* CoinVisualizationFactory::CreateOOBBVisualization(const MathTools::OOBB& oobb, Color colorLine /*= Color::Gray()*/, float lineSize /*= 4.0f*/)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();

        SoSeparator* sep = new SoSeparator();

        std::vector<MathTools::Segment> s = oobb.getSegments();

        for (size_t i = 0; i < s.size(); i++)
        {
            sep->addChild(CreateSegmentVisualization(s[i], colorLine, lineSize));
        }

        res->addChild(sep);
        res->unrefNoDelete();
        return res;
    }

    SoSeparator* CoinVisualizationFactory::CreateSegmentVisualization(const MathTools::Segment& s, Color colorLine /*= Color::Gray()*/, float lineSize /*= 4.0f*/)
    {
        SoSeparator* res = new SoSeparator;
        res->ref();
        SoUnits* u = new SoUnits();
        u->units = SoUnits::MILLIMETERS;
        res->addChild(u);

        SoMaterial* materialLineSolution = new SoMaterial();
        materialLineSolution->ambientColor.setValue(colorLine.r, colorLine.g, colorLine.b);
        materialLineSolution->diffuseColor.setValue(colorLine.r, colorLine.g, colorLine.b);
        SoDrawStyle* lineSolutionStyle = new SoDrawStyle();
        lineSolutionStyle->lineWidth.setValue(lineSize);

        SbVec3f points[2];
        points[0].setValue(s.p0(0), s.p0(1), s.p0(2));
        points[1].setValue(s.p1(0), s.p1(1), s.p1(2));

        res->addChild(lineSolutionStyle);
        res->addChild(materialLineSolution);

        SoCoordinate3* coordinate3 = new SoCoordinate3;
        coordinate3->point.set1Value(0, points[0]);
        coordinate3->point.set1Value(1, points[1]);
        res->addChild(coordinate3);

        SoLineSet* lineSet = new SoLineSet;
        lineSet->numVertices.setValue(2);
        lineSet->startIndex.setValue(0);
        res->addChild(lineSet);

        res->unrefNoDelete();
        return res;
    }


    SoSeparator* CoinVisualizationFactory::CreateEllipse(float x, float y, float z, SoMaterial* matBody, bool showAxes, float axesHeight, float axesWidth, SoMaterial* matX, SoMaterial* matY, SoMaterial* matZ)
    {
        // check for min size
        float minSize = 1e-6f;

        if (x < minSize)
        {
            x = minSize;
        }

        if (y < minSize)
        {
            y = minSize;
        }

        if (z < minSize)
        {
            z = minSize;
        }

        if (!matBody)
        {
            matBody = new SoMaterial;
            matBody->diffuseColor.setValue(0.3f, 0.6f, 0.9f);
            matBody->ambientColor.setValue(0.3f, 0.6f, 0.9f);
            matBody->transparency.setValue(0.3f);
        }

        if (!matX)
        {
            matX = new SoMaterial;
            matX->diffuseColor.setValue(1.0f, 0.2f, 0.2f);
            matX->ambientColor.setValue(1.0f, 0.2f, 0.2f);
            matX->transparency.setValue(0);
        }

        if (!matY)
        {
            matY = new SoMaterial;
            matY->diffuseColor.setValue(0.2f, 0.9f, 0.2f);
            matY->ambientColor.setValue(0.2f, 0.9f, 0.2f);
            matY->transparency.setValue(0);
        }

        if (!matZ)
        {
            matZ = new SoMaterial;
            matZ->diffuseColor.setValue(0.2f, 0.2f, 0.9f);
            matZ->ambientColor.setValue(0.2f, 0.2f, 0.9f);
            matZ->transparency.setValue(0);
        }

        SoSeparator* result = new SoSeparator;
        result->ref();

        // ensure good quality
        SoComplexity* c = new SoComplexity();
        c->type.setValue(SoComplexity::OBJECT_SPACE);
        c->value.setValue(1.0f);
        result->addChild(c);


        SoSeparator* p1 = new SoSeparator;
        result->addChild(p1);

        p1->addChild(matBody);
        SoScale* sc1 = new SoScale;
        sc1->scaleFactor.setValue(x, y, z);
        p1->addChild(sc1);

        SoSphere* sp = new SoSphere();
        sp->radius.setValue(1.0f);
        p1->addChild(sp);

        if (showAxes)
        {

            // y axis
            SoSeparator* ax1 = new SoSeparator();
            result->addChild(ax1);
            ax1->addChild(c);
            SoScale* scAx1 = new SoScale();
            scAx1->scaleFactor.setValue(x + axesHeight, 1.0f, z + axesHeight);
            ax1->addChild(scAx1);
            ax1->addChild(matY);
            SoCylinder* c1 = new SoCylinder();
            c1->radius.setValue(1.0f);
            c1->height.setValue(axesWidth); // cone is aligned with y axis
            ax1->addChild(c1);

            // z axis
            SoSeparator* ax2 = new SoSeparator();
            result->addChild(ax2);
            ax2->addChild(c);
            SoScale* scAx2 = new SoScale();
            scAx2->scaleFactor.setValue(x + axesHeight, y + axesHeight, 1.0f);
            ax2->addChild(scAx2);
            ax2->addChild(matZ);
            SoRotationXYZ* rot2 = new SoRotationXYZ();
            rot2->axis.setValue(SoRotationXYZ::X);
            rot2->angle.setValue(float(M_PI * 0.5f));
            ax2->addChild(rot2);
            SoCylinder* c2 = new SoCylinder();
            c2->radius.setValue(0.999f); // avoid artefacts at meeting points with other axes
            c2->height.setValue(axesWidth);
            ax2->addChild(c2);

            // x axis
            SoSeparator* ax3 = new SoSeparator();
            result->addChild(ax3);
            ax3->addChild(c);
            SoScale* scAx3 = new SoScale();
            scAx3->scaleFactor.setValue(1.0f, y + axesHeight, z + axesHeight);
            ax3->addChild(scAx3);
            ax3->addChild(matX);
            SoRotationXYZ* rot3 = new SoRotationXYZ();
            rot3->axis.setValue(SoRotationXYZ::Z);
            rot3->angle.setValue(float(-M_PI * 0.5f));
            ax3->addChild(rot3);
            SoCylinder* c3 = new SoCylinder();
            c3->radius.setValue(0.998f); // avoid artefacts at meeting points with other axes
            c3->height.setValue(axesWidth);
            ax3->addChild(c3);
        }

        result->unrefNoDelete();
        return result;
    }

    void CoinVisualizationFactory::applyDisplacement(VisualizationNodePtr o, Eigen::Matrix4f& m)
    {
        if (!o)
        {
            return;
        }


        if (o->getType() != getName())
        {
            VR_ERROR << "Skipping Visualization type " << o->getType() << ", but factory is of type " << getName() << endl;
            return;
        }

        CoinVisualizationNode* cvn = dynamic_cast<CoinVisualizationNode*>(o.get());

        if (cvn)
        {
            SoNode* n = cvn->getCoinVisualization();

            if (n)
            {
                SoSeparator* s = new SoSeparator;
                s->ref();
                SoMatrixTransform* ma = getMatrixTransform(m);
                s->addChild(ma);
                s->addChild(n->copy(FALSE));

                cvn->setVisualization(s);
                //o.reset(new CoinVisualizationNode(s));
                s->unrefNoDelete();
            }
        }
        else
        {
            VR_WARNING << "Invalid type casting to CoinVisualizationNode?!" << endl;
        }
    }


    SoGroup* CoinVisualizationFactory::convertSoFileChildren(SoGroup* orig)
    {
        if (!orig)
        {
            return new SoGroup;
        }

        SoGroup* storeResult;

        if (orig->getTypeId() == SoSeparator::getClassTypeId())
        {
            storeResult = new SoSeparator;
        }
        else
        {
            storeResult = new SoGroup;
        }

        storeResult->ref();

        if (orig->getTypeId().isDerivedFrom(SoGroup::getClassTypeId()))
        {
            // process group node
            for (int i = 0; i < orig->getNumChildren(); i++)
            {
                SoNode* n1 = orig->getChild(i);

                if (n1->getTypeId().isDerivedFrom(SoGroup::getClassTypeId()))
                {
                    // convert group
                    SoGroup* n2 = (SoGroup*)n1;
                    SoGroup* gr1 = convertSoFileChildren(n2);
                    storeResult->addChild(gr1);
                }
                else if (n1->getTypeId() == SoFile::getClassTypeId())
                {
                    // really load file!!
                    SoFile* fn = (SoFile*)n1;
                    SoGroup* fileChildren;
                    fileChildren = fn->copyChildren();
                    storeResult->addChild(fileChildren);
                }
                else
                {
                    // just copy child node
                    storeResult->addChild(n1);
                }
            }
        }

        storeResult->unrefNoDelete();
        return storeResult;
    }

    SoNode *CoinVisualizationFactory::copyNode(SoNode *n)
    {
        if (!n)
            return NULL;
        bool copyImages = true;
        std::vector<SoSFImage *> changedImages;
        if( copyImages )
        {
            // find all SoTexture2 nodes
            SoSearchAction search;
            search.setType( SoTexture2::getClassTypeId());
            search.setInterest( SoSearchAction::ALL );
            search.setSearchingAll( TRUE );
            search.apply( n );
            SoPathList & list = search.getPaths();

            //VR_INFO << "copy: copying " <<  list.getLength() << " textures" << std::endl;

            // set their images to not default to copy the contents
            for( int i = 0; i < list.getLength(); i++ )
            {
                SoFullPath * path = (SoFullPath *) list[i];
                assert( path->getTail()->isOfType( SoTexture2::getClassTypeId()));
                SoSFImage * image = &((SoTexture2 *)path->getTail())->image;
                if(image->isDefault() == TRUE)
                {
                    ((SoTexture2 *)path->getTail())->image.setDefault( FALSE );
                    changedImages.push_back( image );
                }
            }

        }
        // the actual copy operation
        SoNode * result = n->copy(TRUE);
        // reset the changed ones back
        for( std::vector<SoSFImage *>::iterator it = changedImages.begin();
             it != changedImages.end(); it++ )
        {
            (*it)->setDefault( TRUE );
        }
        return result;
    }

    SoNode* CoinVisualizationFactory::getColorNode(Color color)
    {
        SoMaterial* m = new SoMaterial();
        m->ambientColor.setValue(color.r, color.g, color.b);
        m->diffuseColor.setValue(color.r, color.g, color.b);
        return m;
    }



} // namespace VirtualRobot
