#ifndef __INVENTOR_COLLADA__
#define __INVENTOR_COLLADA__

#include "collada.h"
#include <Inventor/nodes/SoSeparator.h>

namespace Collada
{
    struct InventorRobotNode : ColladaRobotNode
    {
        SoSeparator* visualization;
        SoSeparator* collisionModel;
        SoSeparator* root;
        SoGroup* preJointTransformation;
        InventorRobotNode(SoSeparator* root);
        InventorRobotNode();
        ~InventorRobotNode();
        void visualizeBoundingBox();
        virtual void initialize();
    private:
        bool m_bOwn;
    };

    // Adds a <instance_geometry_node> directly to a SoSeparator.
    void addGeometry(SoSeparator* separator, const pugi::xml_node& node);

    struct InventorWalker : ColladaWalker
    {
        InventorWalker(StructureMap _structureMap, XmlMap physicsMap, SoSeparator* _root) : ColladaWalker(_structureMap, physicsMap), root(_root) {}
        virtual bool for_each(pugi::xml_node& node);

        SoSeparator* root;
        std::vector<SoSeparator*> stack;
        ColladaRobotNodeSet parents;

    };

    /// Only used for stand-alone Inventor viewer (e.g. for use with SoQtExaminarViewer)
    class InventorRobot : public ColladaRobot
    {
    private:
    public:
        InventorRobot(SoSeparator* _root) : root(_root) {}
        virtual ColladaRobotNodePtr robotNodeFactory()
        {
            return ColladaRobotNodePtr(new InventorRobotNode(root));
        }
        virtual ColladaWalkerPtr visualSceneWalkerFactory(StructureMap structureMap, XmlMap physicsMap)
        {
            return ColladaWalkerPtr(new InventorWalker(structureMap, physicsMap, root));
        }
        virtual void addCollisionModel(ColladaRobotNodePtr robotNode, pugi::xml_node shapeNode);
    protected:
        SoSeparator* root;
        InventorRobot() {};
    };
}//namespace

#endif
