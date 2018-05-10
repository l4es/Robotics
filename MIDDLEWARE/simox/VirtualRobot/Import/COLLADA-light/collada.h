#ifndef __COLLADA_SIMOX_IMPORT_H__
#define __COLLADA_SIMOX_IMPORT_H__


#include "pugixml/pugixml.hpp"
#include <map>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/foreach.hpp>
#endif


#define COLLADA_IMPORT_USE_SENSORS

namespace Collada
{


    typedef std::vector<pugi::xml_node> XmlNodeVector;

    struct ColladaRobotNode;
    typedef boost::shared_ptr<ColladaRobotNode> ColladaRobotNodePtr;
    typedef std::vector<ColladaRobotNodePtr> ColladaRobotNodeSet;
    typedef std::map<pugi::xml_node, ColladaRobotNodePtr> StructureMap;
    typedef std::map<pugi::xml_node, pugi::xml_node> XmlMap;
    typedef std::map<pugi::xml_node, std::vector<pugi::xml_node> > XmlVectorMap;

    struct ColladaRobotNode
    {
        typedef enum
        {
            ePRISMATIC,
            eREVOLUTE
        } JointType;
        JointType type;
        std::string name;
        float value;
        pugi::xml_node joint_axis;
        pugi::xml_node motion_info;
        pugi::xml_node kinematics_info;
        XmlNodeVector rigidBodies;
#ifdef COLLADA_IMPORT_USE_SENSORS
        XmlNodeVector sensors;
#endif
        ColladaRobotNodeSet children;
        ColladaRobotNodePtr parent;
        XmlNodeVector preJoint;

        virtual void initialize() = 0;
    };

    /// Basse class for traversing tree structures in the Collada document (visual and kinematics models)
    struct ColladaWalker : pugi::xml_tree_walker
    {
        ColladaWalker(StructureMap structureMap, XmlMap physicsMap) : structureMap(structureMap), physicsMap(physicsMap) {}
        virtual bool for_each(pugi::xml_node& node) = 0;
        StructureMap structureMap;
        XmlMap physicsMap;
    };
    typedef boost::shared_ptr<ColladaWalker> ColladaWalkerPtr;


    class ColladaRobot
    {
    public:

        void parse(std::string fileName);
        ColladaRobotNodePtr getRoot();
        ColladaRobotNodePtr getNode(std::string name);
        ColladaRobotNodeSet getNodeSet();
        virtual ColladaRobotNodePtr robotNodeFactory() = 0;
        virtual ColladaWalkerPtr visualSceneWalkerFactory(StructureMap structureMap, XmlMap physicsMap) = 0;
        /// Reads an <instance_geometry> in <rigid_body><shape>
        virtual void addCollisionModel(ColladaRobotNodePtr robotNode, pugi::xml_node shapeNode) = 0;

    protected:
        std::string name;
        ColladaRobotNodeSet robotNodeSet;
    };




    // Helper functions to resolve

    /** @brief Resolves the reference in an xml elment's attribute (e.g., `url`).
     * @param node A pugi::xml_node element with either an `url`, `source` or `target` attribute.
     * @param root Restricts the search for the target element (usually a library or an ancestor node).
     * This can double the speed in large files! Defaults to the root node (`<collada>`).
     */
    pugi::xml_node resolveURL(pugi::xml_node node, std::string root = "/*[1]");

    /** @brief Resolves a SIDREF within the document.
     * @param node Any pugi::xml_node element belonging to the document (required for attaining the root node).
     * @param sidref The SIDREF obtained from a node (e.g., `sidref_node.child_value()`)
     * @param root Restricts the search for the target elements (usually a library or an ancestor node).
     * The search for an element might have to include multiple libraries (e.g.,`"//library_articulated_systems|//library_kinematics_models"`) .
     * Defaults to the root node (`<collada>`).
     *
     * A SIDREF is a reference to a child of a node with an `id` tag. The structure is `id/[sid/]*`. However (and this seems not
     * to be clearly stated in the COLLADA specs), if one of the selected child elements is an instance node (e.g., `<instance_joint>`),
     * the instance is resolved and the search is continued in the referee node.
     */
    pugi::xml_node resolveSIDREF(pugi::xml_node node, std::string sidref, std::string root = "/*[1]");

    /// Obtains a vector by casting each of the strings values (separated by spaces)
    template<typename T>
    std::vector<T> getVector(std::string text)
    {
        std::vector<std::string> splitted;
        boost::algorithm::trim(text);
        std::vector<T> result;
        boost::algorithm::split(splitted, text, boost::algorithm::is_space());
        BOOST_FOREACH(std::string number, splitted)
        result.push_back(boost::lexical_cast<T>(number));
        return result;
    }

    template<typename T>
    struct TraversalStack
    {
        std::vector<std::pair<unsigned int, T> > stack;
        inline void push_back(unsigned int depth, T t)
        {
            stack.push_back(std::make_pair(depth, t));
        }

        inline void pop_back(unsigned int depth)
        {
            while (depth > stack.back().first)
            {
                stack.pop_back();
            }
        }
        /// Make sure that depth is greater or equal than the depth of the last element on the stack.
        inline T back()
        {
            return stack.back().second;
        }
    };

};

#endif
