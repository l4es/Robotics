/**
* @package    VirtualRobot
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010 Manfred Kroehnert, Nikolaus Vahrenkamp
*/

#include "CoinVisualizationNode.h"
#include "CoinVisualizationFactory.h"
#include "../../MathTools.h"
#include "../TriMeshModel.h"
#include "../../VirtualRobotException.h"
#include "../../XML/BaseIO.h"

#include <Inventor/SoPrimitiveVertex.h>
#include <Inventor/SbLinear.h>
#include <Inventor/nodes/SoShape.h>
#include <Inventor/nodes/SoNode.h>
#include <Inventor/actions/SoCallbackAction.h>
#include <Inventor/actions/SoLineHighlightRenderAction.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoScale.h>
#include <Inventor/actions/SoWriteAction.h>

#include <Inventor/actions/SoToVRML2Action.h>

#include <Inventor/VRMLnodes/SoVRMLGroup.h>




namespace VirtualRobot
{

    /**
     * Store a reference to \p visualizationNode in the member
     * CoinVisualizationNode::visualization.
     * If \p visualizationNode is a valid object call SoNode::ref() on it.
     */
    CoinVisualizationNode::CoinVisualizationNode(SoNode* visualizationNode) :
        visualization(visualizationNode)
    {
        visualizationAtGlobalPose = new SoSeparator();
        visualizationAtGlobalPose->ref();

        globalPoseTransform = new SoMatrixTransform();
        visualizationAtGlobalPose->addChild(globalPoseTransform);

        attachedVisualizationsSeparator = new SoSeparator();
        attachedVisualizationsSeparator->ref();
        visualizationAtGlobalPose->addChild(attachedVisualizationsSeparator);


        if (!visualization)
        {
            cout << "dummy node created" << endl;
            visualization = new SoSeparator(); // create dummy node
        }

        visualization->ref();
        scaledVisualization = new SoSeparator;
        scaledVisualization->ref();
        scaling = new SoScale;
        scaling->scaleFactor.setValue(1.0f, 1.0f, 1.0f);
        scaledVisualization->addChild(scaling);
        scaledVisualization->addChild(visualization);
        visualizationAtGlobalPose->addChild(scaledVisualization);

    }


    /**
     * If CoinVisualizationNode::visualization is a valid object call SoNode::unref()
     * on it.
     */
    CoinVisualizationNode::~CoinVisualizationNode()
    {
        if (visualization)
        {
            visualization->unref();
        }

        if (attachedVisualizationsSeparator)
        {
            attachedVisualizationsSeparator->unref();
        }

        if (visualizationAtGlobalPose)
        {
            visualizationAtGlobalPose->unref();
        }

        if (scaledVisualization)
        {
            scaledVisualization->unref();
        }
    }

    /*
    void CoinVisualizationNode::convertInventorGroup(SoGroup* orig, SoGroup *storeResult)
    {
        if (orig->getTypeId().isDerivedFrom(SoGroup::getClassTypeId()))
        {
            // process group node
            for (int i=0;i<orig->getNumChildren();i++)
            {
                SoNode* n1 = orig->getChild(i);
                if (n1->getTypeId().isDerivedFrom(SoGroup::getClassTypeId()))
                {
                    // convert group
                    SoGroup *n2 = (SoGroup*)n1;
                    SoGroup *gr1 = new SoGroup();
                    convertInventorGroup (n2,gr1);
                    storeResult->addChild(gr1);
                } else if (n1->getTypeId() == SoFile::getClassTypeId())
                {
                    // really load file!!
                    SoFile *fn = (SoFile*)n1;
                    SoGroup *fileChildren;
                    fileChildren = fn->copyChildren();
                    storeResult->addChild(fileChildren);
                } else
                {
                    // just copy child node
                    storeResult->addChild(n1);
                }
            }
    }*/

    /**
     * This method returns CoinVisualizationNode::triMeshModel.
     * If the model doesn't exist construct it by calling
     * CoinVisualizationNode::createTriMeshModel().
     */
    TriMeshModelPtr CoinVisualizationNode::getTriMeshModel()
    {
        if (!triMeshModel)
        {
            createTriMeshModel();
        }

        return triMeshModel;
    }


    /**
     * This method constructs an instance of TriMeshModel and stores it in
     * CoinVisualizationNode::triMeshModel.
     * If CoinVisualizationMode::visualization is invalid VirtualRobotException
     * is thrown.
     * Otherwise CoinVisualizationNode::InventorTriangleCB() is called on the
     * Inventor graph stored in CoinVisualizationNode::visualization.
     */
    void CoinVisualizationNode::createTriMeshModel()
    {
        THROW_VR_EXCEPTION_IF(!visualization, "CoinVisualizationNode::createTriMeshModel(): no Coin model present!");

        if (triMeshModel)
        {
            triMeshModel->clear();
        }
        else
        {
            triMeshModel.reset(new TriMeshModel());
        }

        SoCallbackAction ca;
        ca.addTriangleCallback(SoShape::getClassTypeId(), &CoinVisualizationNode::InventorTriangleCB, triMeshModel.get());
        ca.apply(visualization);
    }


    /**
     * This method extracts the triangle given by \p v1, \p v2, \p v3 and stores
     * it in the TriMeshModel instance passed in through \p data by calling
     * TriMeshModel::addTriangleWithFace() with the extracted triangle.
     */
    void CoinVisualizationNode::InventorTriangleCB(void* data, SoCallbackAction* action,
            const SoPrimitiveVertex* v1,
            const SoPrimitiveVertex* v2,
            const SoPrimitiveVertex* v3)
    {
        TriMeshModel* triangleMeshModel = static_cast<TriMeshModel*>(data);

        if (!triangleMeshModel)
        {
            VR_INFO << ": Internal error, NULL data" << endl;
            return;
        }

        SbMatrix mm = action->getModelMatrix();
        SbMatrix scale;
        scale.setScale(1000.0f); // simox operates in mm, coin3d in m
        mm = mm.multRight(scale);
        SbVec3f triangle[3];
        mm.multVecMatrix(v1->getPoint(), triangle[0]);
        mm.multVecMatrix(v2->getPoint(), triangle[1]);
        mm.multVecMatrix(v3->getPoint(), triangle[2]);
        SbVec3f normal[3];
        /*mm.multVecMatrix(v1->getNormal(), normal[0]);
        mm.multVecMatrix(v2->getNormal(), normal[1]);
        mm.multVecMatrix(v3->getNormal(), normal[2]);*/
        mm.multDirMatrix(v1->getNormal(), normal[0]);
        mm.multDirMatrix(v2->getNormal(), normal[1]);
        mm.multDirMatrix(v3->getNormal(), normal[2]);

        normal[0] = (normal[0] + normal[1] + normal[2]) / 3.0f;


        // read out vertices
        Eigen::Vector3f a, b, c, n;
        a << triangle[0][0], triangle[0][1], triangle[0][2];
        b << triangle[1][0], triangle[1][1], triangle[1][2];
        c << triangle[2][0], triangle[2][1], triangle[2][2];
        n << normal[0][0], normal[0][1], normal[0][2];
        // add new triangle to the model
        triangleMeshModel->addTriangleWithFace(a, b, c, n);
    }


    /**
     * This mehtod returns the internal CoinVisualizationNode::visualization.
     */
    SoNode* CoinVisualizationNode::getCoinVisualization()
    {
        return visualizationAtGlobalPose;
    }

    void CoinVisualizationNode::setGlobalPose(const Eigen::Matrix4f& m)
    {
        globalPose = m;

        if (globalPoseTransform && updateVisualization)
        {
            SbMatrix m(reinterpret_cast<SbMat*>(globalPose.data()));
            // mm -> m
            m[3][0] *= 0.001f;
            m[3][1] *= 0.001f;
            m[3][2] *= 0.001f;
            globalPoseTransform->matrix.setValue(m);
        }
    }

    void CoinVisualizationNode::print()
    {
        cout << "  CoinVisualization: ";

        if (!triMeshModel)
        {
            createTriMeshModel();
        }

        if (triMeshModel)
        {
            Eigen::Vector3f mi;
            Eigen::Vector3f ma;

            if (triMeshModel->faces.size() > 0)
            {
                cout << triMeshModel->faces.size() << " triangles" << endl;// Extend: " << ma[0]-mi[0] << ", " << ma[1] - mi[1] << ", " << ma[2] - mi[2] << endl;
                triMeshModel->getSize(mi, ma);
                cout << "    Min point: (" << mi[0] << "," << mi[1] << "," << mi[2] << ")" << endl;
                cout << "    Max point: (" << ma[0] << "," << ma[1] << "," << ma[2] << ")" << endl;
            }
            else
            {
                cout << "No model" << endl;
            }

        }
        else
        {
            cout << "No model" << endl;
        }
    }

    void CoinVisualizationNode::attachVisualization(const std::string& name, VisualizationNodePtr v)
    {
        VisualizationNode::attachVisualization(name, v);

        boost::shared_ptr<CoinVisualizationNode> coinVisualizationNode = boost::dynamic_pointer_cast<CoinVisualizationNode>(v);

        if (coinVisualizationNode && coinVisualizationNode->getCoinVisualization())
        {
            attachedCoinVisualizations[name] = coinVisualizationNode->getCoinVisualization();
            attachedVisualizationsSeparator->addChild(coinVisualizationNode->getCoinVisualization());
        }
    }

    void CoinVisualizationNode::detachVisualization(const std::string& name)
    {
        VisualizationNode::detachVisualization(name);
        std::map< std::string, SoNode* >::const_iterator i = attachedCoinVisualizations.begin();

        while (i != attachedCoinVisualizations.end())
        {
            if (i->first == name)
            {
                attachedVisualizationsSeparator->removeChild(i->second);
                attachedCoinVisualizations.erase(name);
                return;
            }

            i++;
        }
    }


    VirtualRobot::VisualizationNodePtr CoinVisualizationNode::clone(bool deepCopy, float scaling)
    {
        THROW_VR_EXCEPTION_IF(scaling <= 0, "Scaling must be >0");

        SoSeparator* newModel = NULL;

        if (visualization)
        {
            newModel = new SoSeparator;
            newModel->ref();

            if (scaling != 1.0)
            {
                SoScale* s = new SoScale;
                s->scaleFactor.setValue(scaling, scaling, scaling);
                newModel->addChild(s);
            }

            if (deepCopy)
            {
                SoNode* deepCopiedNode = CoinVisualizationFactory::copyNode(visualization);
                newModel->addChild(deepCopiedNode);
            }
            else
            {
                newModel->addChild(visualization);
            }
        }

        VisualizationNodePtr p(new CoinVisualizationNode(newModel));

        if (newModel)
        {
            newModel->unrefNoDelete();
        }

        p->setUpdateVisualization(updateVisualization);
        p->setGlobalPose(getGlobalPose());
        p->setFilename(filename, boundingBox);

        // clone attached visualizations
        std::map< std::string, VisualizationNodePtr >::const_iterator i = attachedVisualizations.begin();

        while (i != attachedVisualizations.end())
        {
            VisualizationNodePtr attachedClone = i->second->clone(deepCopy, scaling);
            p->attachVisualization(i->first, attachedClone);
            i++;
        }

        p->primitives = primitives;

        return p;
    }

    void CoinVisualizationNode::setupVisualization(bool showVisualization, bool showAttachedVisualizations)
    {
        VisualizationNode::setupVisualization(showVisualization, showAttachedVisualizations);

        if (!visualizationAtGlobalPose || !attachedVisualizationsSeparator || !visualization)
        {
            return;
        }

        if (showAttachedVisualizations && visualizationAtGlobalPose->findChild(attachedVisualizationsSeparator) < 0)
        {
            visualizationAtGlobalPose->addChild(attachedVisualizationsSeparator);
        }

        if (!showAttachedVisualizations && visualizationAtGlobalPose->findChild(attachedVisualizationsSeparator) >= 0)
        {
            visualizationAtGlobalPose->removeChild(attachedVisualizationsSeparator);
        }


        if (showVisualization && visualizationAtGlobalPose->findChild(scaledVisualization) < 0)
        {
            visualizationAtGlobalPose->addChild(scaledVisualization);
        }

        if (!showVisualization && visualizationAtGlobalPose->findChild(scaledVisualization) >= 0)
        {
            visualizationAtGlobalPose->removeChild(scaledVisualization);
        }
    }

    void CoinVisualizationNode::setVisualization(SoNode* newVisu)
    {
        if (!newVisu)
        {
            return;
        }

        if (scaledVisualization)
        {
            int indx = scaledVisualization->findChild(visualization);

            if (indx >= 0)
            {
                scaledVisualization->removeChild(indx);
            }

        }

        visualization->unref();

        visualization = newVisu;

        visualization->ref();

        if (scaledVisualization)
        {
            scaledVisualization->addChild(visualization);
        }
    }



    bool CoinVisualizationNode::saveModel(const std::string& modelPath, const std::string& filename)
    {
        std::string outFile = filename;
        bool vrml = true; // may be changed later according to file extension

        boost::filesystem::path completePath(modelPath);
        boost::filesystem::path fn(outFile);

        if (!boost::filesystem::is_directory(completePath))
        {
            if (!boost::filesystem::create_directories(completePath))
            {
                VR_ERROR << "Could not create model dir  " << completePath.string() << endl;
                return false;
            }
        }

        boost::filesystem::path completeFile = boost::filesystem::operator/(completePath, fn);

        SoOutput* so = new SoOutput();

        if (!so->openFile(completeFile.string().c_str()))
        {
            VR_ERROR << "Could not open file " << completeFile.string() << " for writing." << endl;
        }

        boost::filesystem::path extension = completeFile.extension();
        std::string extStr = extension.string();
        BaseIO::getLowerCase(extStr);
        if (extStr == ".iv")
            vrml = false;
        else
            vrml = true;


        SoGroup* n = new SoGroup;
        n->ref();
        n->addChild(visualization);
        SoGroup* newVisu = CoinVisualizationFactory::convertSoFileChildren(n);
        newVisu->ref();
        if (vrml)
        {
            SoToVRML2Action tovrml2;
            tovrml2.apply(newVisu);
            SoVRMLGroup *newroot = tovrml2.getVRML2SceneGraph();
            newroot->ref();
            so->setHeaderString("#VRML V2.0 utf8");
            SoWriteAction wra(so);
            wra.apply(newroot);
            newroot->unref();
        }
        else
        {
            SoWriteAction wa(so);
            wa.apply(newVisu);
        }
        so->closeFile();

        newVisu->unref();
        n->unref();

        return true;
    }

    void CoinVisualizationNode::scale(Eigen::Vector3f& scaleFactor)
    {
        scaling->scaleFactor.setValue(scaleFactor(0), scaleFactor(1), scaleFactor(2));
        triMeshModel.reset();
    }

} // namespace VirtualRobot
