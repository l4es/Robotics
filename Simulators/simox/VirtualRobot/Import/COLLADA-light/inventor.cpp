#include "inventor.h"




#include <Inventor/nodes/SoSphere.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoRotation.h>
#include <Inventor/nodes/SoTranslation.h>
#include <Inventor/nodes/SoMatrixTransform.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoIndexedFaceSet.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoPendulum.h>
#include <Inventor/nodes/SoRotor.h>
#include <Inventor/nodes/SoCube.h>
#include <Inventor/nodes/SoTransform.h>
#include <Inventor/nodes/SoDrawStyle.h>


#include <Inventor/actions/SoGetBoundingBoxAction.h>

#include <iostream>

#ifdef TIMER_DEBUG
#ifndef Q_MOC_RUN
#include <boost/timer/timer.hpp>
#endif
#endif

using namespace std;

namespace Collada
{


    void addTransform(SoGroup* root, pugi::xml_node node)
    {
        if (std::string("rotate").compare(node.name()) == 0)
        {
            SoRotation* transform = new SoRotation;
            root->addChild(transform);
            std::vector<float> v = getVector<float>(node.child_value());
            transform->rotation.setValue(SbVec3f(v[0], v[1], v[2]), v[3]);
        }
        else if (std::string("translate").compare(node.name()) == 0)
        {
            SoTranslation* transform = new SoTranslation;
            root->addChild(transform);
            std::vector<float> v = getVector<float>(node.child_value());
            transform->translation.setValue(v[0], v[1], v[2]);
        }
        else if (std::string("matrix").compare(node.name()) == 0)
        {
            SoMatrixTransform* transform = new SoMatrixTransform;
            root->addChild(transform);
            std::vector<float> v = getVector<float>(node.child_value());
            transform->matrix.setValue(SbMatrix(
                                           v[0],    v[4],    v[8],   v[12],
                                           v[1],    v[5],    v[9],   v[13],
                                           v[2],    v[6],   v[10],   v[14],
                                           v[3],    v[7],   v[11],   v[15]));
        }
    }

    InventorRobotNode::InventorRobotNode()
    {
        m_bOwn = false;
    }

    InventorRobotNode::InventorRobotNode(SoSeparator* _root)
    {
        m_bOwn = true;
        visualization = new SoSeparator;
        visualization->ref();
        preJointTransformation = new SoGroup;
        visualization->addChild(preJointTransformation);
        root = _root;
        collisionModel = new SoSeparator;
        collisionModel->ref();
    }

    InventorRobotNode::~InventorRobotNode()
    {
        if (m_bOwn)
        {
            visualization->unref();
            collisionModel->unref();
        }
    }

    void InventorRobotNode::visualizeBoundingBox()
    {

        SbViewportRegion vpReg;
        vpReg.setWindowSize(300, 200);
        SoGetBoundingBoxAction getBBoxAction(vpReg);
        getBBoxAction.apply(this->visualization);
        SoSeparator* separator = new SoSeparator;
        this->visualization->insertChild(separator, 0);
        SoDrawStyle* drawstyle = new SoDrawStyle;
        separator->addChild(drawstyle);
        drawstyle->style.setValue(SoDrawStyleElement::LINES);

        if (getBBoxAction.getBoundingBox().hasVolume())
        {
            SoTransform* center = new SoTransform;
            SbVec3f bounds = getBBoxAction.getBoundingBox().getMax() - getBBoxAction.getBoundingBox().getMin();
            separator->addChild(center);
            center->translation = getBBoxAction.getBoundingBox().getCenter();
            SoCube*  BB = new SoCube;
            separator->addChild(BB);
            BB->width.setValue(bounds[0]);
            BB->height.setValue(bounds[1]);
            BB->depth.setValue(bounds[2]);
        }
    }

    void InventorRobotNode::initialize()
    {
        if (this->parent)
        {
            cout << "Parent of " << this->name << " is " << this->parent->name << endl;
            boost::dynamic_pointer_cast<InventorRobotNode>(this->parent)->visualization->addChild(this->visualization);
        }
        else
        {
            root->addChild(this->collisionModel);
            root->addChild(this->visualization);
        }

        BOOST_FOREACH(pugi::xml_node node, this->preJoint)
        {
            addTransform(this->preJointTransformation, node);
        }
        std::string name = std::string("RobotNode_") + this->name;
        this->visualization->setName(name.c_str());
        this->preJointTransformation->setName("PreJointTransformations");


        SoSphere* sphere = new SoSphere;
        this->preJointTransformation->addChild(sphere);
        sphere->radius = 0.01f;
        /* SoPendulum seems broken :(
        SoPendulum *pendulum = new SoPendulum;
        this->preJointTransformation->addChild(pendulum);
        vector<float> v = getVector<float>(this->joint_axis.child_value("axis"));
        pendulum->rotation0.setValue(SbVec3f(0,0,1),0);
        pendulum->rotation1.setValue(SbVec3f(1,0,0),3.14);
        //pendulum->rotation.setValue(SbVec3f(1,0,0),0);
        pendulum->speed = 0.01;
        pendulum->on = true;*/

    }

    template<typename T>
    std::vector<T> offsetSelection(std::vector<T> v, int offset, int stride)
    {
        assert(v.size() % stride == 0);
        vector<T> result(v.size() / stride);

        // classical for-loop is faster than iterators and push_back()
        //int j=0;
        for (int i = offset, j = 0; i < v.size(); i += stride, j++)
        {
            result[j] = v[i];
        }

        return result;
    }

    std::map<std::string, std::vector<float> > addMaterials(const pugi::xml_node& igeometry)
    {
        std::map<std::string, std::vector<float> > colormap;
        BOOST_FOREACH(pugi::xpath_node imaterial, igeometry.select_nodes(".//instance_material"))
        {
            std::string symbol = imaterial.node().attribute("symbol").value();
            pugi::xml_node material = resolveURL(imaterial.node(), "//library_materials");
            pugi::xml_node ieffect = material.child("instance_effect");
            pugi::xml_node effect = resolveURL(ieffect, "//library_effects");
            pugi::xml_node phong = effect.child("profile_COMMON").child("technique").child("phong");
            //        pugi::xml_node diffuse = phong.child("specular").child("color");
            pugi::xml_node diffuse = phong.child("diffuse").child("color");
            colormap[symbol] = getVector<float>(diffuse.child_value());
        }
        return colormap;
    }

    void addGeometry(SoSeparator* separator, const pugi::xml_node& node)
    {

        std::map<std::string, std::vector<float> > colormap = addMaterials(node);
        pugi::xml_node geometry = resolveURL(node, "//library_geometries");
        pugi::xml_node mesh = geometry.child("mesh");

#ifdef TIMER_DEBUG
        boost::timer::auto_cpu_timer timer(1, string("Inventor (") + string(geometry.attribute("id").value()) + string("):  %t sec CPU, %w sec real\n"));
        cout << "Faces: " ;
#endif

        // Temporarily skip complicated models
        int nPolylist = mesh.select_nodes(".//polylist").size();

        if (nPolylist > 5)
        {
            cout << "More than five (" << nPolylist << ") seperated meshes .. skipping (" << geometry.attribute("id").value() << ")\n";
            //                return;
        }

        BOOST_FOREACH(pugi::xpath_node polylist, mesh.select_nodes(".//polylist"))
        {
            //std::cout << "id: " << geometry.attribute("id").value() << std::endl;
            std::vector<float> color;

            if (polylist.node().attribute("material") && polylist.node().attribute("material").value() && colormap.size()>0)
            {
                color = colormap.at(polylist.node().attribute("material").value());
            }
            else
            {
                color = std::vector<float>(3, 0.5);
            }

            SoMaterial* mat  = new SoMaterial;
            separator->addChild(mat);
            mat->diffuseColor.setValue(color[0], color[1], color[2]);
            //std::cout << "color: " << color[0] << "," << color[1] << "," << color[2] << "," << color[3] << std::endl;
            //mat->transparency.setValue(color[3]);

            SoCoordinate3* coordinates = new SoCoordinate3;
            separator->addChild(coordinates);
            SoIndexedFaceSet* faceSet = new SoIndexedFaceSet;
            separator->addChild(faceSet);
            //faceSet->coordIndex.setValues(0,p2.size(),&p2[0]);


            std::vector<int> p = getVector<int>(polylist.node().child_value("p"));
            std::vector<int> vcount = getVector<int>(polylist.node().child_value("vcount"));
#ifdef TIMER_DEBUG
            cout << vcount.size() << ", " << flush;
#endif
            pugi::xml_node vertexInputNode = polylist.node().select_single_node(".//input[@semantic='VERTEX']").node();

            // get number of input nodes
            int stride = polylist.node().select_nodes(".//input").size();
            // Get the vertex indices
            int vertexOffset = boost::lexical_cast<int>(vertexInputNode.attribute("offset").value());
            //vector<int> vertexIndices(p.size()/stride+vcount.size());

            pugi::xml_node vertices = resolveURL(vertexInputNode);
            pugi::xml_node source = resolveURL(vertices.child("input"));
            std::vector<float> floats = getVector<float>(source.child_value("float_array"));
            coordinates->point.setValues(0, floats.size() / 3, (const float(*)[3]) &floats[0]);

            /* Explanation:
             * Collada stores indices in one single vector: (vertex1, normal1, color1, vertex2 ...)
             * Inventor stores vertices in a vector with a -1 between vertices of different faces: (vertex11, vertex12, vertex13, -1, vertex21, vertex22, ...)
             * This leads to the follwing loop that creates the vertex indices for InventorRobot
             */
            for (int pInd = vertexOffset, vOffset = 0, vInd = 0, countInd = 0; pInd < int(p.size()); pInd += stride, vInd++)
            {
                if (vInd >= vcount[countInd])
                {
                    //vertexIndices[vInd+vOffset] = SO_END_FACE_INDEX;
                    faceSet->coordIndex.set1Value(vInd + vOffset, SO_END_FACE_INDEX);
                    vOffset += vInd + 1;
                    countInd++;
                    vInd = 0;
                }

                faceSet->coordIndex.set1Value(vInd + vOffset, p[pInd]);
                //vertexIndices[vInd+vOffset] = p[pInd];
                //assert(vertexIndices[vInd+vOffset]<=floats.size()/3);
            }


        }
#ifdef TIMER_DEBUG
        cout << endl;
#endif
    }


    bool InventorWalker::for_each(pugi::xml_node& node)
    {
        if (depth() + 1 > int(stack.size())) {}

        while (depth() + 1 < int(stack.size()))
        {
            stack.pop_back();
            parents.pop_back();
        }

        if (parents.size() == 0)
        {
            parents.push_back(ColladaRobotNodePtr());
        }

        if (std::string("node").compare(node.name()) == 0)
        {
            SoSeparator* separator = new SoSeparator;

            if (depth() == 0)
            {
                root->addChild(separator);
            }
            else
            {
                stack.back()->addChild(separator);
            }

            stack.push_back(separator);
            separator->setName(SbName(node.attribute("id").value()));
        }
        else if (std::string("rotation").compare(node.name()) == 0)
        {
            addTransform(stack.back(), node);
        }
        else if (std::string("translation").compare(node.name()) == 0)
        {
            addTransform(stack.back(), node);
        }
        else if (std::string("matrix").compare(node.name()) == 0)
        {
            addTransform(stack.back(), node);
        }
        else if (std::string("instance_geometry").compare(node.name()) == 0)
        {
            addGeometry(stack.back(), node);
        }

        if (structureMap.count(node))
        {
            stack.back() = boost::dynamic_pointer_cast<InventorRobotNode>(structureMap[node])->visualization;
            parents.push_back(structureMap[node]);
        }
        else
        {
            parents.push_back(parents.back());
        }

        if (physicsMap.count(node))
        {
            parents.back()->rigidBodies.push_back(physicsMap[node]);
        }

        return true;
    }

    void InventorRobot::addCollisionModel(ColladaRobotNodePtr robotNode, pugi::xml_node RigidBodyNode)
    {

        boost::shared_ptr<InventorRobotNode> inventorRobotNode = boost::dynamic_pointer_cast<InventorRobotNode>(robotNode);
        SoSeparator* rigidBodySep = new SoSeparator;
        inventorRobotNode->collisionModel->addChild(rigidBodySep);
        // An additional Separator is necessary if there are multiple rigid bodies attached to the same joint


        BOOST_FOREACH(pugi::xpath_node transformation, RigidBodyNode.select_nodes(".//mass_frame/translate|.//mass_frame/rotate"))
        addTransform(rigidBodySep, transformation.node());

        BOOST_FOREACH(pugi::xpath_node shape, RigidBodyNode.select_nodes(".//shape"))
        {
            SoSeparator* separator  = new SoSeparator;
            rigidBodySep->addChild(separator);
            BOOST_FOREACH(pugi::xpath_node transformation, shape.node().select_nodes(".//translate|.//rotate"))
            addTransform(separator, transformation.node());
            addGeometry(separator, shape.node().child("instance_geometry"));
        }
    }





} // namespace
