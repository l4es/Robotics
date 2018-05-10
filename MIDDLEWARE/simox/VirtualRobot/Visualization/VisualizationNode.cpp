/**
* @package    VirtualRobot
* @author     Manfred Kroehnert
* @author     Nikolaus Vahrenkamp
* @copyright  2010 Manfred Kroehnert
*/


#include "VisualizationNode.h"
#include "TriMeshModel.h"

#include "VirtualRobot/VirtualRobot.h"
#include "VirtualRobot/VirtualRobotException.h"
#include "VirtualRobot/XML/BaseIO.h"


namespace VirtualRobot
{

    VisualizationNode::VisualizationNode()
    {
        updateVisualization = true;
        showVisualization = true;
        showAttachedVisualizations = true;
        boundingBox = false;
        globalPose.setIdentity();
    }

    VisualizationNode::~VisualizationNode()
    {
        attachedVisualizations.clear();
    }

    VirtualRobot::VisualizationNodePtr VisualizationNode::clone(bool deepCopy, float scaling)
    {
        VisualizationNodePtr p(new VisualizationNode());
        p->setUpdateVisualization(updateVisualization);
        p->setFilename(filename, boundingBox);
        p->primitives = primitives;

        return p;
    }

    VirtualRobot::TriMeshModelPtr VisualizationNode::getTriMeshModel()
    {
        return TriMeshModelPtr();
    }

    void VisualizationNode::attachVisualization(const std::string& name, VisualizationNodePtr v)
    {
        THROW_VR_EXCEPTION_IF(!v, "NULL DATA");

        attachedVisualizations[name] = v;
    }

    void VisualizationNode::detachVisualization(const std::string& name)
    {
        attachedVisualizations.erase(name);
    }

    bool VisualizationNode::hasAttachedVisualization(const std::string& name)
    {
        std::map< std::string, VisualizationNodePtr >::const_iterator i = attachedVisualizations.begin();

        while (i != attachedVisualizations.end())
        {
            if (i->first == name)
            {
                return true;
            }

            i++;
        }

        return false;
    }

    VisualizationNodePtr VisualizationNode::getAttachedVisualization(const std::string& name)
    {
        if (!hasAttachedVisualization(name))
        {
            return VisualizationNodePtr();
        }

        return attachedVisualizations[name];
    }


    void VisualizationNode::setUpdateVisualization(bool enable)
    {
        updateVisualization = enable;
    }

    bool VisualizationNode::getUpdateVisualizationStatus()
    {
        return updateVisualization;
    }

    void VisualizationNode::print()
    {
        cout << "Dummy VisualizationNode" << endl;
    }

    void VisualizationNode::setupVisualization(bool showVisualization, bool showAttachedVisualizations)
    {
        this->showVisualization = showVisualization;
        this->showAttachedVisualizations = showAttachedVisualizations;
    }

    int VisualizationNode::getNumFaces()
    {
        TriMeshModelPtr p = getTriMeshModel();

        if (p)
        {
            return p->faces.size();
        }

        return 0;
    }

    void VisualizationNode::setFilename(const std::string& filename, bool boundingBox)
    {
        this->filename = filename;
        this->boundingBox = boundingBox;
    }

    bool VisualizationNode::usedBoundingBoxVisu()
    {
        return boundingBox;
    }

    std::string VisualizationNode::getFilename()
    {
        return filename;
    }

    std::string VisualizationNode::toXML(const std::string& basePath, const std::string& filename, int tabs)
    {
        std::stringstream ss;
        std::string t = "\t";
        std::string pre = "";

        for (int i = 0; i < tabs; i++)
        {
            pre += "\t";
        }

        ss << pre << "<Visualization";

        if (usedBoundingBoxVisu())
        {
            ss << " BoundingBox='true'";
        }

        ss << ">\n";

        if (!filename.empty())
        {
            std::string tmpFilename = filename;
            BaseIO::makeRelativePath(basePath, tmpFilename);
            ss << pre << t << "<File type='" << getType() << "'>" << tmpFilename << "</File>\n";
        }
        else if (primitives.size() != 0)
        {
            ss << pre << "\t<Primitives>\n";
            std::vector<Primitive::PrimitivePtr>::const_iterator it;

            for (it = primitives.begin(); it != primitives.end(); it++)
            {
                ss << (*it)->toXMLString(tabs + 1);
            }

            ss << pre << "\t</Primitives>\n";
        }

        ss << pre << "</Visualization>\n";

        return ss.str();
    }

    std::string VisualizationNode::toXML(const std::string& basePath, int tabs)
    {
        std::string visualizationFilename = getFilename();
        boost::filesystem::path fn(visualizationFilename);
        return toXML(basePath, fn.string(), tabs);
    }

    VirtualRobot::VisualizationNodePtr VisualizationNode::CreateUnitedVisualization(const std::vector<VisualizationNodePtr>& visualizations)
    {
        if (visualizations.size() == 0)
        {
            return VisualizationNodePtr();
        }

        VisualizationFactoryPtr f;
        std::vector<VisualizationNodePtr>::const_iterator i = visualizations.begin();

        while (!f && i != visualizations.end())
        {
            if ((*i)->getType() != VisualizationFactory::getName())
            {
                f = VisualizationFactory::fromName((*i)->getType(), NULL);
                break;
            }

            i++;
        }

        if (i == visualizations.end())
        {
            VR_ERROR << "Could not find visualization factory. Aborting..." << endl;
            return VisualizationNodePtr();
        }

        THROW_VR_EXCEPTION_IF(!f, "No VisualizationFactory");

        return f->createUnitedVisualization(visualizations);
    }

    VirtualRobot::BoundingBox VisualizationNode::getBoundingBox()
    {
        VirtualRobot::BoundingBox bbox;
        TriMeshModelPtr tm = getTriMeshModel();

        if (!tm)
        {
            return bbox;
        }

        bbox = tm->boundingBox;
        bbox.transform(globalPose);
        return bbox;
    }

    bool VisualizationNode::saveModel(const std::string& modelPath, const std::string& filename)
    {
        // derived classes have to overwrite this method, otherwise a NYI will show up
        VR_ERROR << "NYI..." << endl;
        return false;
    }

    void VisualizationNode::scale(Eigen::Vector3f& scaleFactor)
    {
        VR_WARNING << "not implemented..." << endl;
    }

    void VisualizationNode::createTriMeshModel()
    {

    }

} // namespace VirtualRobot
