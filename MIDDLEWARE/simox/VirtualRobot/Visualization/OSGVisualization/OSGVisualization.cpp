/**
* @package    VirtualRobot
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*/


#include "OSGVisualization.h"
#include "OSGVisualizationNode.h"

#include <algorithm>

namespace VirtualRobot
{

    OSGVisualization::OSGVisualization(const VisualizationNodePtr visualizationNode) :
        Visualization(visualizationNode)
    {
        visu = NULL;
    }

    OSGVisualization::OSGVisualization(const std::vector<VisualizationNodePtr>& visualizationNodes) :
        Visualization(visualizationNodes)
    {
        visu = NULL;
    }

    OSGVisualization::~OSGVisualization()
    {
        if (visu)
        {
            visu->unref();
        }
    }

    bool OSGVisualization::buildVisualization()
    {
        if (visu)
        {
            return true;
        }

        visu = new osg::Group;
        visu->ref();

        BOOST_FOREACH(VisualizationNodePtr visualizationNode, visualizationNodes)
        {
            boost::shared_ptr<OSGVisualizationNode> osgNode = boost::dynamic_pointer_cast<OSGVisualizationNode>(visualizationNode);

            if (osgNode && osgNode->getOSGVisualization())
            {
                visu->addChild(osgNode->getOSGVisualization());
            }
        }
        return true;
    }

    bool OSGVisualization::highlight(unsigned int which, bool enable)
    {
        if (which >= visualizationNodes.size())
        {
            VR_WARNING << "Could not find visualizationNode..." << endl;
            return false;
        }

        return highlight(visualizationNodes[which], enable);
    }


    bool OSGVisualization::highlight(osg::Node* visu, bool enable)
    {
        if (!visu)
        {
            return false;
        }

        /*if (enable)
            selection->select(visu);
        else
            selection->deselect(visu);

        selection->touch();*/
        VR_WARNING << "NYI..." << endl;
        return true;
    }

    bool OSGVisualization::highlight(VisualizationNodePtr visualizationNode, bool enable)
    {
        if (!isVisualizationNodeRegistered(visualizationNode))
        {
            VR_WARNING << "Could not find visualizationNode..." << endl;
            return false;
        }

        boost::shared_ptr<OSGVisualizationNode> osgNode = boost::dynamic_pointer_cast<OSGVisualizationNode>(visualizationNode);

        if (osgNode)
        {
            return highlight(osgNode->getOSGVisualization(), enable);
        }

        return false;
    }

    bool OSGVisualization::highlight(bool enable)
    {
        for (size_t i = 0; i < visualizationNodes.size(); i++)
        {
            highlight(i, enable);
        }

        return true;
    }

    /**
     * This method iterates over the entries in member
     * OSGVisualization::visualizationNodes and stores the return value of
     * OSGVisualizationNode::getOSGVisualization() in an SoSeparator if the
     * processed node is of type OSGVisualizationNode.
     * Afterwards the SoSeparator is returned.
     */
    osg::Node* OSGVisualization::getOSGVisualization()
    {
        buildVisualization();
        return visu;
    }

    /**
     * \return new instance of VirtualRobot::OSGVisualization with the same set of robot nodes.
     */
    VirtualRobot::VisualizationPtr OSGVisualization::clone()
    {
        return VisualizationPtr(new OSGVisualization(visualizationNodes));
    }


} // namespace VirtualRobot
