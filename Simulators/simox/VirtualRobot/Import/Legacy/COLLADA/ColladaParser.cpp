/**
 * This file is part of Simox.
 *
 * Simox is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * Simox is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * @package    VirtualRobot
 * @author     Stefan Ulbrich
 * @copyright  Stefan Ulbrich
 *             GNU Lesser General Public License
 */

#include "ColladaParser.h"
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <sstream>

using namespace std;
using namespace ColladaDOM150;

namespace VirtualRobot
{

    template<class T>
    ostream& operator<<(ostream& os, const vector<T>& vec)
    {
        boost::copy(vec, std::ostream_iterator<T>(os, ","));
        return os;
    }

    template<class T>
    ostream& operator<<(ostream& os, const vector<vector<T> >& vec)
    {
        os << "[";
        BOOST_FOREACH(vector<T> nested, vec)
        {
            os << "[";
            boost::copy(nested, std::ostream_iterator<T>(os, ","));
            os << "]";
        }
        os << "]";
        return os;
    }

    ostream& operator<<(ostream& os, const ColladaParser::SceneGraph::GeometryType& gt)
    {
        os << gt.vertices << endl << gt.triangles << endl;
        return os;
    }

    ostream& operator<<(ostream& os, const ColladaParser::SceneGraph& dt)
    {
        BOOST_FOREACH(vector<float> transformation, dt.transformations)
        {
            os << (transformation.size() == 3 ? "Translation: " : "Rotation: ") << transformation << endl;
        }
        os << dt.geometries << endl;
        BOOST_FOREACH(boost::shared_ptr<ColladaParser::SceneGraph> child, dt.children)
        os << *(child) << endl;
        return os;
    }

    /// Output to the command line for debugging.
    ostream& operator<<(ostream& os, const ColladaParser::NodeData& dt)
    {
        os << boost::format("Name: %s, Speed: %f, Acceleration: %f, Deceleration: %f, Jerk: %f, Active: %s, Locked: %s, Min: %f, Max: %f") % dt.name.c_str()
           % dt.speed % dt.acceleration % dt.deceleration % dt.jerk % (dt.active ? "True" : "False") % (dt.locked ? "True" : "False") % dt.min % dt.max;
        BOOST_FOREACH(vector<float> transformation, dt.transformations)
        {
            os << (transformation.size() == 3 ? "Translation: " : "Rotation: ") << transformation << endl;
        }
        BOOST_FOREACH(boost::shared_ptr<ColladaParser::NodeData> sensor, dt.sensors)
        {
            os << "**** BEGIN SENSOR *****" << endl;
            os << *(sensor);
            os << "**** END SENSOR *****" << endl;
        }

        if (dt.sceneGraph)
        {
            os << "\nScene graph: " << *(dt.sceneGraph) << endl;
        }

        BOOST_FOREACH(boost::shared_ptr<ColladaParser::NodeData> child, dt.children)
        os << *(child);
        return os;
    }

    /// Output to the command line for debugging.
    template<class T>
    ostream& operator<<(ostream& os, const daeTArray<T>& array)
    {
        boost::copy(array, std::ostream_iterator<T>(os, ","));
        return os;
    }


    void ColladaParser::extractSensors(domJoint* actualJoint, boost::shared_ptr<ColladaParser::NodeData> jointNode, domArticulated_system* motion_articulated_system)
    {
        BOOST_FOREACH(domExtraRef extra, motion_articulated_system->getExtra_array())
        {
            if (strcmp(extra->getType(), "attach_sensor") == 0)
            {
                string name(extra->getName());
                //cout << name ;
                boost::shared_ptr<ColladaParser::NodeData> node(new ColladaParser::NodeData());
                node->name = name;
                domJoint* joint;

                BOOST_FOREACH(domTechniqueRef technique, extra->getTechnique_array())
                {
                    if (strcmp(technique->getProfile(), "OpenRAVE") == 0)
                    {
                        BOOST_FOREACH(daeElementRef elt, technique->getContents())
                        {
                            if (strcmp(elt->getElementName(), "frame_origin") == 0)
                            {
                                {
                                    daeSidRef sidref(std::string(elt->getAttribute("link")), motion_articulated_system);
                                    assert(sidref.resolve().elt);
                                    daeElement* parent = sidref.resolve().elt->getParentElement();
                                    //cout << parent->getElementName() << endl;
                                    daeSidRef sidref2(std::string(parent->getAttribute("joint")), motion_articulated_system);
                                    assert(sidref2.resolve().elt);
                                    //cout << dynamic_cast<domInstance_joint*>(sidref2.resolve().elt)->getUrl().getElement()->getElementName() << endl;
                                    joint  = dynamic_cast<domJoint*>(dynamic_cast<domInstance_joint*>(sidref2.resolve().elt)->getUrl().getElement().cast());
                                    assert(joint);
                                }
                                //cout << elt->getAttribute("link") << endl;
                                BOOST_FOREACH(daeElementRef child, elt->getChildren())
                                {
                                    std::vector<float> trafo;
                                    istringstream charData(child->getCharData());

                                    //cout << child->getElementName() << ": " << endl;
                                    if (strcmp(child->getElementName(), "translate") | strcmp(child->getElementName(), "rotate"))
                                    {
                                        std::copy(std::istream_iterator<float>(charData), std::istream_iterator<float>(),  std::back_inserter(trafo));
                                    }

                                    node->transformations.push_back(trafo);
                                    //cout << trafo << trafo.size() << endl;
                                }
                            }
                        }
                    }
                }

                if (joint == actualJoint)
                {
                    // TODO Create if necessary.
                    jointNode->sensors.push_back(node);
                }
            }
        }
    }

    ColladaParser::ColladaParser(string filename)
    {


        daeElement* root = this->dae.open(filename.c_str());

        if (!root)
        {
            cout << "Document import failed.\n";
        }


    }


    boost::shared_ptr<ColladaParser::SceneGraph> ColladaParser::parseVisualScene(domNode* node, std::vector<daeElement*> delimiters, daeElement* start)
    {
        assert(node);
        //cout << node->getID() << endl;
        bool found = false;

        if (!start)
        {
            found = true;
        }

        boost::shared_ptr<ColladaParser::SceneGraph> sceneGraph(new ColladaParser::SceneGraph);

        BOOST_FOREACH(daeElementRef elt, node->getContents())
        {
            if (elt.cast() == start)
            {
                found = true;
            }

            if (found)   // Skip the elements before start
            {

                if (boost::find(delimiters, elt) != delimiters.end())
                {
                    return sceneGraph;    //return if delimiter is reached
                }

                daeInt ID = elt->typeID();
                vector<float> transformation;

                if (ID == domMatrix::ID())
                {
                    domFloat3 values = dynamic_cast<domMatrix*>(elt.cast())->getValue();
                    //cout << "Translation: " << values << endl;
                    // Convert daeTArray to std::vector
                    transformation.resize(16);
                    boost::copy(values, transformation.begin());
                    sceneGraph->transformations.push_back(transformation);
                }
                else if (ID == domTranslate::ID())
                {
                    domFloat3 values = dynamic_cast<domTranslate*>(elt.cast())->getValue();
                    //cout << "Translation: " << values << endl;
                    // Convert daeTArray to std::vector
                    transformation.resize(3);
                    boost::copy(values, transformation.begin());
                    sceneGraph->transformations.push_back(transformation);
                }
                else if (ID == domRotate::ID())
                {
                    domFloat4 values = dynamic_cast<domRotate*>(elt.cast())->getValue();
                    // Convert daeTArray to std::vector
                    transformation.resize(4);
                    boost::copy(values, transformation.begin());
                    sceneGraph->transformations.push_back(transformation);
                }
                else if (ID == domNode::ID())
                {
                    domNode* childNode = dynamic_cast<domNode*>(elt.cast());

                    if (rigid_body_map.find(childNode) != rigid_body_map.end())
                    {
                        sceneGraph->rigidBodies.push_back(rigid_body_map.at(childNode));
                    }

                    sceneGraph->children.push_back(parseVisualScene(childNode, delimiters));
                }
                else if (ID == domScale::ID())
                {
                    vector<float> scales(3);
                    domFloat3 values = dynamic_cast<domScale*>(elt.cast())->getValue();
                    boost::copy(values, scales.begin());
                    transformation = vector<float>(16, 0);
                    transformation[0] = scales[0];
                    transformation[5] = scales[1];
                    transformation[10] = scales[2];
                    transformation[15] = 1;
                    sceneGraph->transformations.push_back(transformation);
                }
                else if (ID == domInstance_geometry::ID())
                {
                    ColladaParser::SceneGraph::GeometryType sg_geometry;

                    domInstance_geometry* instance_geometry = dynamic_cast<domInstance_geometry*>(elt.cast());
                    BOOST_FOREACH(domInstance_materialRef matRef, instance_geometry->getBind_material()->getTechnique_common()->getInstance_material_array())
                    {
                        ColladaParser::SceneGraph::GeometryType::PhongMaterial phongMaterial;
                        domMaterial* material = dynamic_cast<domMaterial*>(matRef->getTarget().getElement().cast());
                        phongMaterial.id = material->getId();
                        domEffect* effect = dynamic_cast<domEffect*>(material->getInstance_effect()->getUrl().getElement().cast());
                        assert(effect->getFx_profile_array().getCount() == 1);
                        domFx_profile* p = effect->getFx_profile_array()[0];
                        //                    domProfile_common* profile = dynamic_cast<domProfile_common>(p->getProfile_COMMON());
                        domProfile_common* profile = p->getProfile_COMMON();
                        assert(profile);
                        domProfile_common::domTechnique::domPhong* phong = profile->getTechnique()->getPhong();
                        assert(phong);

                        phongMaterial.ambient.resize(4, 0.0f);
                        phongMaterial.diffuse.resize(4, 0.0f);
                        phongMaterial.emission.resize(4, 0.0f);
                        phongMaterial.specular.resize(4, 0.0f);
                        phongMaterial.reflective.resize(4, 0.0f);
                        phongMaterial.transparent.resize(4, 0.0f);

                        if (phong->getAmbient())
                        {
                            boost::copy(phong->getAmbient()->getColor()->getValue(), phongMaterial.ambient.begin());
                        }

                        if (phong->getDiffuse())
                        {
                            boost::copy(phong->getDiffuse()->getColor()->getValue(), phongMaterial.diffuse.begin());
                        }

                        if (phong->getEmission())
                        {
                            boost::copy(phong->getEmission()->getColor()->getValue(), phongMaterial.emission.begin());
                        }

                        if (phong->getSpecular())
                        {
                            boost::copy(phong->getSpecular()->getColor()->getValue(), phongMaterial.specular.begin());
                        }

                        if (phong->getReflective())
                        {
                            boost::copy(phong->getReflective()->getColor()->getValue(), phongMaterial.reflective.begin());
                        }

                        if (phong->getTransparent())
                        {
                            boost::copy(phong->getTransparent()->getColor()->getValue(), phongMaterial.transparent.begin());
                        }

                        phongMaterial.shininess = (phong->getShininess()) ? phong->getShininess()->getFloat()->getValue() : 0;
                        phongMaterial.reflectivity = (phong->getReflectivity()) ? phong->getReflectivity()->getFloat()->getValue() : 0;
                        phongMaterial.transparency = (phong->getTransparency()) ? phong->getTransparency()->getFloat()->getValue() : 0;
                        phongMaterial.refractionIndex = (phong->getIndex_of_refraction()) ? phong->getIndex_of_refraction()->getFloat()->getValue() : 0;

                        sg_geometry.materials.push_back(phongMaterial);
                    }

                    domGeometry* geometry = dynamic_cast<domGeometry*>(instance_geometry->getUrl().getElement().cast());
                    domMeshRef mesh = geometry->getMesh();
                    BOOST_FOREACH(domSourceRef source, mesh->getSource_array()) {}
                    domVerticesRef vertices = mesh->getVertices();

                    //TODO fix redundancy of polyLists/triangles
                    BOOST_FOREACH(domTrianglesRef triangles, mesh->getTriangles_array())
                    {
                        daeUInt inputCount = triangles->getInput_array().getCount();
                        daeUInt vertexOffset = -1;
                        daeUInt normalOffset = -1;
                        daeUInt colorOffset = -1;
                        daeUInt materialRef = -1;

                        std::string materialID = triangles->getMaterial();

                        for (size_t i = 0; i < sg_geometry.materials.size(); i++)
                        {
                            if (materialID == sg_geometry.materials[i].id)
                            {
                                materialRef = i;
                                break;
                            }
                        }

                        assert(materialRef != -1);

                        BOOST_FOREACH(domInput_local_offsetRef input, triangles->getInput_array())
                        {
                            if (strcmp(input->getSemantic(), "VERTEX") == 0)
                            {
                                vertexOffset = input->getOffset();
                                domVertices* vertices = dynamic_cast<domVertices*>(input->getSource().getElement().cast());
                                assert(vertices);
                                BOOST_FOREACH(domInput_localRef input, vertices->getInput_array())
                                {
                                    if (strcmp(input->getSemantic(), "POSITION") == 0)
                                    {
                                        domSource* source = dynamic_cast<domSource*>(input->getSource().getElement().cast());
                                        assert(source);
                                        domAccessorRef accessor = source->getTechnique_common()->getAccessor();
                                        daeUInt count = daeUInt(accessor->getCount());
                                        assert(source->getFloat_array()->getValue().getCount());
                                        daeUInt stride = daeUInt(accessor->getStride());
                                        //cout << "#Points: " << count /*/ stride*/ << endl;
                                        assert(stride == 3);

                                        for (unsigned int i = 0; i < count/*/stride*/; i++)
                                        {
                                            vector<float> point(stride);

                                            for (unsigned int j = 0; j < stride; j++)
                                            {
                                                //assert(i*stride+j < count);
                                                point[j] = float(source->getFloat_array()->getValue()[i * stride + j]);
                                            }

                                            sg_geometry.vertices.push_back(point);
                                        }
                                    }
                                }
                            }
                            else if (strcmp(input->getSemantic(), "NORMAL") == 0)
                            {
                                normalOffset = daeUInt(input->getOffset());
                                domSource* source = dynamic_cast<domSource*>(input->getSource().getElement().cast());
                                assert(source);
                                domAccessorRef accessor = source->getTechnique_common()->getAccessor();
                                daeUInt count = daeUInt(accessor->getCount());
                                assert(source->getFloat_array()->getValue().getCount());
                                daeUInt stride = daeUInt(accessor->getStride());
                                //cout << "#Normals: " << count << endl;
                                assert(stride == 3);

                                for (unsigned int i = 0; i < count; i++)
                                {
                                    vector<float> point(stride);

                                    for (unsigned int j = 0; j < stride; j++)
                                    {
                                        point[j] = float(source->getFloat_array()->getValue()[i * stride + j]);
                                    }

                                    sg_geometry.normals.push_back(point);
                                }
                            }
                            else if (strcmp(input->getSemantic(), "COLOR") == 0)
                            {
                                colorOffset = daeUInt(input->getOffset());
                                domSource* source = dynamic_cast<domSource*>(input->getSource().getElement().cast());
                                assert(source);
                                domAccessorRef accessor = source->getTechnique_common()->getAccessor();
                                daeUInt count = daeUInt(accessor->getCount());
                                assert(source->getFloat_array()->getValue().getCount());
                                daeUInt stride = daeUInt(accessor->getStride());
                                //cout << "#Colors: " << count << endl;
                                assert(stride == 3);

                                for (unsigned int i = 0; i < count; i++)
                                {
                                    vector<float> point(stride);

                                    for (unsigned int j = 0; j < stride; j++)
                                    {
                                        point[j] = float(source->getFloat_array()->getValue()[i * stride + j]);
                                    }

                                    sg_geometry.colors.push_back(point);
                                }
                            }
                        }

                        if (vertexOffset != -1)
                        {
                            vector<unsigned int> temp(triangles->getP()->getValue().getCount() / inputCount);

                            for (size_t i = vertexOffset, j = 0; i < triangles->getP()->getValue().getCount(); i += inputCount, j++)
                            {
                                temp[j] = triangles->getP()->getValue()[i];
                            }

                            //                      boost::copy(triangles->getP()->getValue(),temp.begin());
                            sg_geometry.triangles.push_back(temp);
                            sg_geometry.triangleMaterial.push_back(materialRef);
                        }

                        if (normalOffset != -1)
                        {
                            vector<unsigned int> temp(triangles->getP()->getValue().getCount() / inputCount);

                            for (size_t i = normalOffset, j = 0; i < triangles->getP()->getValue().getCount(); i += inputCount, j++)
                            {
                                temp[j] = triangles->getP()->getValue()[i];
                            }

                            sg_geometry.triangleNormals.push_back(temp);
                        }

                        if (colorOffset != -1)
                        {
                            vector<unsigned int> temp(triangles->getP()->getValue().getCount() / inputCount);

                            for (size_t i = colorOffset, j = 0; i < triangles->getP()->getValue().getCount(); i += inputCount, j++)
                            {
                                temp[j] = triangles->getP()->getValue()[i];
                            }

                            sg_geometry.triangleColors.push_back(temp);
                        }
                    }

                    BOOST_FOREACH(domPolylistRef polyLists, mesh->getPolylist_array())
                    {
                        daeUInt inputCount = polyLists->getInput_array().getCount();
                        daeUInt vertexOffset = -1;
                        daeUInt normalOffset = -1;
                        daeUInt colorOffset = -1;
                        daeUInt materialRef = -1;

                        std::string materialID = polyLists->getMaterial();

                        for (size_t i = 0; i < sg_geometry.materials.size(); i++)
                        {
                            if (materialID == sg_geometry.materials[i].id)
                            {
                                materialRef = i;
                                break;
                            }
                        }

                        assert(materialRef != -1);

                        BOOST_FOREACH(domInput_local_offsetRef input, polyLists->getInput_array())
                        {

                            if (strcmp(input->getSemantic(), "VERTEX") == 0)
                            {
                                vertexOffset = input->getOffset();
                                domVertices* vertices = dynamic_cast<domVertices*>(input->getSource().getElement().cast());
                                assert(vertices);
                                BOOST_FOREACH(domInput_localRef input, vertices->getInput_array())
                                {
                                    if (strcmp(input->getSemantic(), "POSITION") == 0)
                                    {
                                        domSource* source = dynamic_cast<domSource*>(input->getSource().getElement().cast());
                                        assert(source);
                                        domAccessorRef accessor = source->getTechnique_common()->getAccessor();
                                        daeUInt count = accessor->getCount();
                                        assert(source->getFloat_array()->getValue().getCount());
                                        daeUInt stride = accessor->getStride();
                                        //cout << "#Points: " << count /*/ stride*/ << endl;
                                        assert(stride == 3);

                                        for (int i = 0; i < count/*/stride*/; i++)
                                        {
                                            vector<float> point(stride);

                                            for (int j = 0; j < stride; j++)
                                            {
                                                //assert(i*stride+j < count);
                                                point[j] = source->getFloat_array()->getValue()[i * stride + j];
                                            }

                                            sg_geometry.vertices.push_back(point);
                                        }
                                    }
                                }
                            }
                            else if (strcmp(input->getSemantic(), "NORMAL") == 0)
                            {
                                normalOffset = input->getOffset();
                                domSource* source = dynamic_cast<domSource*>(input->getSource().getElement().cast());
                                assert(source);
                                domAccessorRef accessor = source->getTechnique_common()->getAccessor();
                                daeUInt count = accessor->getCount();
                                assert(source->getFloat_array()->getValue().getCount());
                                daeUInt stride = accessor->getStride();
                                //cout << "#Normals: " << count << endl;
                                assert(stride == 3);

                                for (unsigned int i = 0; i < count; i++)
                                {
                                    vector<float> point(stride);

                                    for (unsigned int j = 0; j < stride; j++)
                                    {
                                        point[j] = float(source->getFloat_array()->getValue()[i * stride + j]);
                                    }

                                    sg_geometry.normals.push_back(point);
                                }
                            }
                            else if (strcmp(input->getSemantic(), "COLOR") == 0)
                            {
                                colorOffset = input->getOffset();
                                domSource* source = dynamic_cast<domSource*>(input->getSource().getElement().cast());
                                assert(source);
                                domAccessorRef accessor = source->getTechnique_common()->getAccessor();
                                daeUInt count = accessor->getCount();
                                assert(source->getFloat_array()->getValue().getCount());
                                daeUInt stride = accessor->getStride();
                                //cout << "#Colors: " << count << endl;
                                assert(stride == 3);

                                for (unsigned int i = 0; i < count; i++)
                                {
                                    vector<float> point(stride);

                                    for (unsigned int j = 0; j < stride; j++)
                                    {
                                        point[j] = source->getFloat_array()->getValue()[i * stride + j];
                                    }

                                    sg_geometry.colors.push_back(point);
                                }
                            }
                            else if (strcmp(input->getSemantic(), "TEXCOORD") == 0)
                            {

                            }
                            else
                            {
                                //cout << "NYI: Collada polyLists Semantics =" << input->getSemantic() << endl;
                            }
                        }

                        if (vertexOffset != -1)
                        {
                            vector<unsigned int> temp(polyLists->getP()->getValue().getCount() / inputCount);

                            for (size_t i = vertexOffset, j = 0; i < polyLists->getP()->getValue().getCount(); i += inputCount, j++)
                            {
                                temp[j] = polyLists->getP()->getValue()[i];
                            }

                            //                      boost::copy(polyLists->getP()->getValue(),temp.begin());
                            sg_geometry.triangles.push_back(temp);
                            sg_geometry.triangleMaterial.push_back(materialRef);
                        }

                        if (normalOffset != -1)
                        {
                            vector<unsigned int> temp(polyLists->getP()->getValue().getCount() / inputCount);

                            for (size_t i = normalOffset, j = 0; i < polyLists->getP()->getValue().getCount(); i += inputCount, j++)
                            {
                                temp[j] = polyLists->getP()->getValue()[i];
                            }

                            sg_geometry.triangleNormals.push_back(temp);
                        }

                        if (colorOffset != -1)
                        {
                            vector<unsigned int> temp(polyLists->getP()->getValue().getCount() / inputCount);

                            for (size_t i = colorOffset, j = 0; i < polyLists->getP()->getValue().getCount(); i += inputCount, j++)
                            {
                                temp[j] = polyLists->getP()->getValue()[i];
                            }

                            sg_geometry.triangleColors.push_back(temp);
                        }
                    }

                    sceneGraph->geometries.push_back(sg_geometry);
                }
            }
        }
        //cout << sceneGraph->geometries << endl;
        return sceneGraph;
    }


    bool ColladaParser::parseLink(domLinkRef link, boost::shared_ptr<ColladaParser::NodeData> node, domKinematics_model* kinematics_model, ColladaParser::NodeDataMap& jointMap)
    {
        if (!link)
        {
            return false;
        }

        vector<daeElement*> delimiters; // Stop parsing the visual_scene when reaching the next jont axis
        BOOST_FOREACH(domLink::domAttachment_fullRef attachment_full, link->getAttachment_full_array())
        {
            daeSidRef sidref(attachment_full->getJoint(), kinematics_model);
            daeElement* elt  = sidref.resolve().elt;
            domInstance_joint* instance_joint = dynamic_cast<domInstance_joint*>(sidref.resolve().elt);
            assert(instance_joint);
            domJoint* joint = dynamic_cast<domJoint*>(instance_joint->getUrl().getElement().cast());
            assert(joint);
            jointMap[joint]->name = joint->getID();
            BOOST_FOREACH(daeElementRef elt, attachment_full->getContents())
            {

                daeInt ID = elt->typeID();
                vector<float> transformation;

                if (ID == domMatrix::ID())
                {
                    domFloat3 values = dynamic_cast<domMatrix*>(elt.cast())->getValue();
                    cout << "Translation: " << values << endl;
                    // Convert daeTArray to std::vector
                    transformation.resize(16);
                    boost::copy(values, transformation.begin());
                    jointMap[joint]->transformations.push_back(transformation);
                }
                else if (ID == domTranslate::ID())
                {
                    domFloat3 values = dynamic_cast<domTranslate*>(elt.cast())->getValue();
                    cout << "Translation: " << values << endl;
                    // Convert daeTArray to std::vector
                    transformation.resize(3);
                    boost::copy(values, transformation.begin());
                    jointMap[joint]->transformations.push_back(transformation);
                }
                else if (ID == domRotate::ID())
                {
                    domFloat4 values = dynamic_cast<domRotate*>(elt.cast())->getValue();
                    // Convert daeTArray to std::vector
                    transformation.resize(4);
                    boost::copy(values, transformation.begin());
                    jointMap[joint]->transformations.push_back(transformation);
                }
                else if (ID == domScale::ID())
                {
                    vector<float> scales(3);
                    domFloat3 values = dynamic_cast<domScale*>(elt.cast())->getValue();
                    boost::copy(values, scales.begin());
                    transformation = vector<float>(16, 0);
                    transformation[0] = scales[0];
                    transformation[5] = scales[1];
                    transformation[10] = scales[2];
                    transformation[15] = 1;
                    jointMap[joint]->transformations.push_back(transformation);
                }
            }

            /*
            if (jointMap[joint]->jointType==eRevolute){
                std::vector<float> dof = jointMap[joint]->axis;
                dof.push_back(jointMap[joint]->value);
                jointMap[joint]->transformations.push_back(dof);
            }
            else{
                std::vector<float> dof = jointMap[joint]->axis;
                for (int i =0; i < 3; i++)
                    dof[i]*=jointMap[joint]->value;
                jointMap[joint]->transformations.push_back(dof);
            }*/

            node->children.push_back(jointMap[joint]);
            delimiters.push_back(jointMap[joint]->transformation_visual_scene);
            this->parseLink(attachment_full->getLink(), jointMap[joint], kinematics_model, jointMap);
            //        cout << *node << endl;
        }
        //Now parse the visual scene!
        domNode* visual_parent_node;

        if (node->transformation_visual_scene->typeID() != domNode::ID())
        {
            visual_parent_node = dynamic_cast<domNode*>(node->transformation_visual_scene->getParent());
        }
        else
        {
            visual_parent_node = dynamic_cast<domNode*>(node->transformation_visual_scene);    // Only for the root element (links to a node and not a transformation)
        }

        std::cout << "visual_parent_node" <<     visual_parent_node->getID() << std::endl;
        node->sceneGraph = this->parseVisualScene(visual_parent_node, delimiters, node->transformation_visual_scene);
        return true;
    };

    void ColladaParser::parse()
    {

        domCOLLADA* collada = dae.getDatabase()->typeLookup<domCOLLADA>().at(0);
        //domCOLLADA *collada = dynamic_cast<domCOLLADA*> (root->getDescendant("COLLADA"));
        domCOLLADA::domSceneRef scene = collada->getScene();
        domInstance_with_extraRef instance_visual_scene = scene->getInstance_visual_scene();

        // Get the visual scene
        domVisual_scene* visual_scene = dynamic_cast<domVisual_scene*>(instance_visual_scene->getUrl().getElement().cast());
        assert(visual_scene && "Dynamic cast failed");

        // Parse all defined rigid body definitions and store them in relation to their associated node.

        BOOST_FOREACH(domInstance_with_extra * instance, scene->getInstance_physics_scene_array())
        {
            domPhysics_scene* physics_scene = dynamic_cast<domPhysics_scene*>(instance->getUrl().getElement().cast());
            BOOST_FOREACH(domInstance_physics_model * instance_physics_model, physics_scene->getInstance_physics_model_array())
            {
                domPhysics_model* physics_model = dynamic_cast<domPhysics_model*>(instance_physics_model->getUrl().getElement().cast());
                std::map<std::string, domRigid_body*> rigid_body_name_map;
                BOOST_FOREACH(domRigid_body * rigid_body, physics_model->getRigid_body_array())
                {
                    rigid_body_name_map[rigid_body->getID()] = rigid_body;
                }

                BOOST_FOREACH(domInstance_rigid_body * instance_rigid_body, instance_physics_model->getInstance_rigid_body_array())
                {
                    ColladaParser::SceneGraph::RigidBodyType rigidBodyType;

                    domRigid_body* rigid_body = rigid_body_name_map[instance_rigid_body->getBody()];
                    std::vector<float> inertia;
                    istringstream charData(rigid_body->getTechnique_common()->getInertia()->getCharData());
                    std::copy(std::istream_iterator<float>(charData), std::istream_iterator<float>(),  std::back_inserter(rigidBodyType.inertia));
                    //                        std::cout << rigidBodyType.inertia <<std::endl;
                    rigidBodyType.mass = float(boost::lexical_cast<double>(rigid_body->getTechnique_common()->getMass()->getCharData()));
                    //                        std::cout <<  rigidBodyType.mass << std::endl;
                    BOOST_FOREACH(daeElement * child, rigid_body->getTechnique_common()->getMass_frame()->getContents())
                    {
                        std::vector<float> trafo;
                        istringstream charData(child->getCharData());

                        if (strcmp(child->getElementName(), "translate") | strcmp(child->getElementName(), "rotate"))
                        {
                            std::copy(std::istream_iterator<float>(charData), std::istream_iterator<float>(),  std::back_inserter(trafo));
                        }

                        //                            std::cout << child->getElementName() << " " << trafo << std::endl;
                        rigidBodyType.mass_frame.push_back(trafo);
                    }
                    domNode* node = dynamic_cast<domNode*>(instance_rigid_body->getTarget().getElement().cast());

                    if (node)
                        //                           std::cout << "Node associated\n";
                    {
                        rigid_body_map[node] = rigidBodyType;
                    }
                }

            }

        }


        // get the kinematics scenes
        BOOST_FOREACH(domInstance_kinematics_scene * instance_kinematics_scene, scene->getInstance_kinematics_scene_array())
        {

            domKinematics_scene* kinematics_scene = dynamic_cast<domKinematics_scene*>(instance_kinematics_scene->getUrl().getElement().cast());
            assert(kinematics_scene);
            NodeDataMap jointMap;
            BOOST_FOREACH(domBind_joint_axisRef bind_joint_axis,
                          instance_kinematics_scene->getBind_joint_axis_array())
            {
                daeElement* transformation_visual_scene;
                {
                    daeSidRef sidref(std::string(bind_joint_axis->getTarget()), visual_scene);
                    transformation_visual_scene  = sidref.resolve().elt;
                }
                BOOST_FOREACH(domInstance_articulated_systemRef instance_articulated_system,
                              kinematics_scene->getInstance_articulated_system_array())
                {
                    domArticulated_system* motion_articulated_system = dynamic_cast<domArticulated_system*>(instance_articulated_system->getUrl().getElement().cast());


                    domJoint* joint; // the joint we are looking for

                    domArticulated_system* kinematics_articulated_system = dynamic_cast<domArticulated_system*>(motion_articulated_system->getMotion()->getInstance_articulated_system()->getUrl().getElement().cast());
                    //                domKinematics_model* kinematics_model = dynamic_cast<domKinematics_model*>
                    assert(motion_articulated_system && "kinematics");
                    assert(kinematics_articulated_system && "kinematics");
                    BOOST_FOREACH(domKinematics_bindRef bind, instance_articulated_system->getBind_array())
                    {
                        if (strcmp(bind->getSymbol(), bind_joint_axis->getAxis()->getParam()->getValue()) == 0)
                        {
                            daeSidRef sidref(std::string(bind->getParam()->getRef()), motion_articulated_system);
                            daeElement* joint_axis  = sidref.resolve().elt;
                            joint  = dynamic_cast<domJoint*>(joint_axis->getParent());
                            assert(joint);
                            BOOST_FOREACH(domMotion_axis_infoRef motion_axis_info, motion_articulated_system->getMotion()->getTechnique_common()->getAxis_info_array())
                            {
                                domKinematics_axis_info* kinematics_axis_info;
                                {
                                    daeSidRef sidref(std::string(motion_axis_info->getAxis()), motion_articulated_system);
                                    kinematics_axis_info = dynamic_cast<domKinematics_axis_info*>(sidref.resolve().elt);
                                }
                                assert(kinematics_axis_info);
                                domJoint* target_joint;
                                {
                                    daeSidRef sidref(std::string(kinematics_axis_info->getAxis()), kinematics_articulated_system);
                                    target_joint  = dynamic_cast<domJoint*>(sidref.resolve().elt->getParent());
                                }
                                assert(target_joint);
                                {
                                    daeSidRef sidref(std::string(kinematics_axis_info->getAxis()), NULL);
                                    daeElement* elt  = sidref.resolve().elt;
                                }

                                if (target_joint == joint)
                                {
                                    if (!jointMap[joint])
                                    {
                                        jointMap[joint].reset(new ColladaParser::NodeData());
                                    }

                                    jointMap[joint]->speed = motion_axis_info->getSpeed()->getFloat()->getValue();
                                    jointMap[joint]->acceleration = motion_axis_info->getAcceleration()->getFloat()->getValue();
                                    jointMap[joint]->deceleration = motion_axis_info->getDeceleration()->getFloat()->getValue();
                                    jointMap[joint]->jerk = motion_axis_info->getJerk()->getFloat()->getValue();

                                    jointMap[joint]->locked = kinematics_axis_info->getLocked()->getBool()->getValue();
                                    jointMap[joint]->active = true;// kinematics_axis_info->getActive()->getBool()->getValue();
                                    jointMap[joint]->min = kinematics_axis_info->getLimits()->getMin()->getFloat()->getValue();
                                    jointMap[joint]->max = kinematics_axis_info->getLimits()->getMax()->getFloat()->getValue();
                                    jointMap[joint]->transformation_visual_scene = transformation_visual_scene;
                                    jointMap[joint]->axis.resize(3);

                                    if (dynamic_cast<domAxis_constraint*>(joint_axis) &&  dynamic_cast<domAxis_constraint*>(joint_axis)->getAxis())
                                    {
                                        //boost::copy(dynamic_cast<domAxis_constraint*> (joint_axis) -> getAxis()->getValue(), jointMap[joint]->axis.begin());
                                        domAxisRef a = dynamic_cast<domAxis_constraint*>(joint_axis) -> getAxis();

                                        if (strcmp(joint_axis->getElementName(), "revolute") == 0)
                                        {
                                            jointMap[joint]->jointType = eRevolute;
                                        }
                                        else if (strcmp(joint_axis->getElementName(), "prismatic") == 0)
                                        {
                                            jointMap[joint]->jointType = ePrismatic;
                                        }
                                        else
                                        {
                                            jointMap[joint]->jointType = eNone;
                                        }

                                        domFloat3 f = a->getValue();
                                        jointMap[joint]->axis[0] = float(f[0]);
                                        jointMap[joint]->axis[1] = float(f[1]);
                                        jointMap[joint]->axis[2] = float(f[2]);
                                    }
                                    else
                                    {
                                        jointMap[joint]->axis.resize(3, 0.0f);
                                        jointMap[joint]->jointType = eNone;
                                    }

                                    // Search for VICON Markers
                                    this->extractSensors(joint, jointMap[joint], motion_articulated_system);
                                }
                            }
                            continue;
                        } // if
                    } // FOREACH

                    assert(joint);
                    BOOST_FOREACH(domKinematics_bindRef bind, instance_articulated_system->getBind_array())
                    {

                        if (strcmp(bind->getSymbol(), bind_joint_axis->getValue()->getParam()->getValue()) == 0)
                        {
                            daeSidRef sidref(std::string(bind->getParam()->getRef()), motion_articulated_system);
                            daeElement* element = sidref.resolve().elt;
                            std::cout <<  element->getChildren()[0]->getCharData();
                            domKinematics_newparam* newparam  = dynamic_cast<domKinematics_newparam*>(element);
                            jointMap[joint]->value = newparam->getFloat()->getValue();
                            std::cout << "initial value for: " << joint->getID() << " is " << newparam->getFloat()->getValue() << std::endl;
                            continue;
                        } // if

                    } // FOREACH
                }
            }
            BOOST_FOREACH(domBind_kinematics_modelRef bind_kinematics_model,
                          instance_kinematics_scene->getBind_kinematics_model_array())
            {
                domNode* root_node;
                {
                    daeSidRef sidref(std::string(bind_kinematics_model->getNode()), visual_scene);
                    root_node = dynamic_cast<domNode*>(sidref.resolve().elt);
                }
                BOOST_FOREACH(domInstance_articulated_systemRef instance_articulated_system,
                              kinematics_scene->getInstance_articulated_system_array())
                {
                    domArticulated_system* motion_articulated_system = dynamic_cast<domArticulated_system*>(instance_articulated_system->getUrl().getElement().cast());
                    BOOST_FOREACH(domKinematics_bindRef bind, instance_articulated_system->getBind_array())
                    {
                        if (strcmp(bind->getSymbol(), bind_kinematics_model->getParam()->getValue()) == 0)
                        {
                            daeSidRef sidref(std::string(bind->getParam()->getRef()), motion_articulated_system);
                            daeElement* elt = sidref.resolve().elt;
                            domInstance_kinematics_model* instance_kinematics_model  = dynamic_cast<domInstance_kinematics_model*>(sidref.resolve().elt);
                            assert(instance_kinematics_model);
                            domKinematics_model* kinematics_model = dynamic_cast<domKinematics_model*>(instance_kinematics_model->getUrl().getElement().cast());
                            assert(kinematics_model);
                            assert(kinematics_model->getTechnique_common()->getLink_array().getCount() == 1); // The only supported way
                            BOOST_FOREACH(domLinkRef link, kinematics_model->getTechnique_common()->getLink_array())
                            {
                                this->model[kinematics_scene->getID()].reset(new ColladaParser::NodeData); // The root element
                                this->model[kinematics_scene->getID()]->transformation_visual_scene = root_node;
                                this->parseLink(link, this->model[kinematics_scene->getID()], kinematics_model, jointMap);
                            }
                        }
                    }
                }
            }
        }
        // Import of COLLADA v1.5:
        // Outline:
        // 1) explore the <scene>/<instance_kinematics_scene>, look for bindings of the kinematics model.
        // 2) In <kinematics_scene> the model is either directly instantiated (NOT supported)
        //    or combined with dynamics and/or additional kinematic information (both required).
        //    Only the <articulated_system> with a <motion> element is instantiated in the
        //    <kinematics_scene>. The <articulated_system> with the <kinematics_element> is intatiated
        //    in there. This last element is the place where the <kinematics_model> is finally
        //    intatiated. This second step resolves/finds the correct model.
        // 3) The tree structure of the <kinematics_model> is parsed. Every joint(-axis) encountered
        //    gets its kinematic and dynamic information extracted. This requires visiting the
        //    whole structure up to the <scene>-object. That way the associated <node> of the
        //    current <visual_scene> is found.
        // 4) With the <node> known for each joint, the meshes can be connected. This is done by getting
        //    The parent node of the transformation in the visual scene. Every <instance_geometry> deeper in
        //    the hierarchy is assigned to the joint. This assignment takes place in the recursion of 3)
        //    The assignment consequently gets overriden by child joints. The association is stored in a global
        //    dict that is returned at the end.
        // 5) OpenRAVE requires additional information stored in the <extra>/<technique profile="OpenRAVE">
        //    element. Parameters are stored in local variables/dictionaries. (This step is actually
        //    performed earlier during 2) when the <articulated_system> with <motion> is detected)
        //    Limitations: 1 axis per joint,


    }

    ostream& operator<<(ostream& os, const ColladaParser& parser)
    {
        os << "COLLADA Parser" << std::endl;
        BOOST_FOREACH(const ColladaParser::ModelType::value_type & i, parser.model)
        os << "Model: " << i.first << std::endl << *(i.second) << std::endl;
        return os;
    }

}
