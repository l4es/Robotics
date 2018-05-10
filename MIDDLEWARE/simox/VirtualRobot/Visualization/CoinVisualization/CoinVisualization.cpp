/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @copyright  2010 Manfred Kroehnert
*/


#include "CoinVisualization.h"
#include "CoinVisualizationNode.h"

#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoUnits.h>

#include <algorithm>

// For the VRML2.0 export
#include <Inventor/actions/SoWriteAction.h>
#include <Inventor/actions/SoToVRML2Action.h>
#include <Inventor/VRMLnodes/SoVRMLGroup.h>
#include <Inventor/nodes/SoRotation.h>

namespace VirtualRobot
{

    CoinVisualization::CoinVisualization(const VisualizationNodePtr visualizationNode) :
        Visualization(visualizationNode)
    {
        selection = NULL;
    }

    CoinVisualization::CoinVisualization(const std::vector<VisualizationNodePtr>& visualizationNodes) :
        Visualization(visualizationNodes)
    {
        selection = NULL;
    }

    CoinVisualization::~CoinVisualization()
    {
        if (selection)
        {
            selection->unref();
        }
    }

    bool CoinVisualization::buildVisualization()
    {
        if (selection)
        {
            return true;
        }

        selection = new SoSelection;
        selection->ref();
        selection->policy = SoSelection::TOGGLE;
        SoSeparator* visualization = new SoSeparator();
        //SoMatrixTransform *mtr = new SoMatrixTransform;
        /*SbMatrix m(reinterpret_cast<SbMat*>(globalPose.data()));
        mtr->matrix.setValue(m);
        selection->addChild(mtr);*/
        SoUnits* u = new SoUnits();
        u->units = SoUnits::METERS;
        visualization->addChild(u);
        selection->addChild(visualization);

        BOOST_FOREACH(VisualizationNodePtr visualizationNode, visualizationNodes)
        {
            boost::shared_ptr<CoinVisualizationNode> coinVisualizationNode = boost::dynamic_pointer_cast<CoinVisualizationNode>(visualizationNode);

            if (coinVisualizationNode && coinVisualizationNode->getCoinVisualization())
            {
                visualization->addChild(coinVisualizationNode->getCoinVisualization());
            }
        }
        return true;
    }

    bool CoinVisualization::highlight(unsigned int which, bool enable)
    {
        if (which >= visualizationNodes.size())
        {
            VR_WARNING << "Could not find visualizationNode..." << endl;
            return false;
        }

        return highlight(visualizationNodes[which], enable);
    }

    bool CoinVisualization::highlight(SoNode* visu, bool enable)
    {
        if (!visu)
        {
            return false;
        }

        if (enable)
        {
            selection->select(visu);
        }
        else
        {
            selection->deselect(visu);
        }

        selection->touch();
        return true;
    }

    bool CoinVisualization::highlight(VisualizationNodePtr visualizationNode, bool enable)
    {
        if (!selection)
        {
            return false;
        }

        if (!isVisualizationNodeRegistered(visualizationNode))
        {
            VR_WARNING << "Could not find visualizationNode..." << endl;
            return false;
        }

        boost::shared_ptr<CoinVisualizationNode> coinVisualizationNode = boost::dynamic_pointer_cast<CoinVisualizationNode>(visualizationNode);


        if (coinVisualizationNode)
        {
            return highlight(coinVisualizationNode->getCoinVisualization(), enable);
        }

        return false;
    }

    bool CoinVisualization::highlight(bool enable)
    {
        for (size_t i = 0; i < visualizationNodes.size(); i++)
        {
            highlight(i, enable);
        }

        return true;
    }

    /**
     * This method iterates over the entries in member
     * CoinVisualization::visualizationNodes and stores the return value of
     * CoinVisualizationNode::getCoinVisualization() in an SoSeparator if the
     * processed node is of type CoinVisualizationNode.
     * Afterwards the SoSeparator is returned.
     */
    SoNode* CoinVisualization::getCoinVisualization()
    {
        buildVisualization();
        return selection;
    }

    /**
     * \return new instance of VirtualRobot::CoinVisualization with the same set of robot nodes.
     */
    VirtualRobot::VisualizationPtr CoinVisualization::clone()
    {
        return VisualizationPtr(new CoinVisualization(visualizationNodes));
    }

    void CoinVisualization::exportToVRML2(std::string filename)
    {

        SoSeparator* root = new SoSeparator;
        root->ref();
        SoRotation* rotationX = new SoRotation();
        rotationX->ref();
        rotationX->rotation.setValue(SbVec3f(-1, 0, 0), float(M_PI / 2));
        SoRotation* rotationZ = new SoRotation();
        rotationZ->ref();
        rotationZ->rotation.setValue(SbVec3f(0, 0, 1), float(M_PI));

        root->addChild(rotationX);
        root->addChild(rotationZ);
        root->addChild(this->getCoinVisualization());

        printf("Converting...\n");
        SoToVRML2Action tovrml2;
        tovrml2.apply(root);
        SoVRMLGroup* newroot = tovrml2.getVRML2SceneGraph();
        newroot->ref();
        root->unref();
        rotationZ->unref();
        rotationX->unref();

        printf("Writing...\n");

        SoOutput out;
        out.openFile(filename.c_str());
        out.setHeaderString("#VRML V2.0 utf8");
        SoWriteAction wra(&out);
        wra.apply(newroot);
        out.closeFile();
        newroot->unref();
    }


} // namespace VirtualRobot
