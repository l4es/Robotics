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
* @author     Manfred Kroehnert, Nikolaus Vahrenkamp
* @copyright  2010, 2011 Manfred Kroehnert, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_CoinVisualizationNode_h_
#define _VirtualRobot_CoinVisualizationNode_h_

#include "../../VirtualRobotImportExport.h"
#include "../VisualizationNode.h"
#include "CoinVisualizationFactory.h"



class SoNode;
class SoSeparator;
class SoScale;
class SoCallbackAction;
class SoPrimitiveVertex;
class SoMatrixTransform;

namespace VirtualRobot
{
    class TriMeshModel;

    class VIRTUAL_ROBOT_IMPORT_EXPORT CoinVisualizationNode : virtual public VisualizationNode
    {
        friend class CoinVisualizationFactory;
    public:
        CoinVisualizationNode(SoNode* visualizationNode);
        ~CoinVisualizationNode();
        virtual TriMeshModelPtr getTriMeshModel();

        SoNode* getCoinVisualization();

        virtual void setGlobalPose(const Eigen::Matrix4f& m);

        virtual void print();

        virtual void scale(Eigen::Vector3f& scaleFactor);

        /*!
            Attach an optional visualization to this VisualizationNode. The attached visualizations will not show up in the TriMeshModel.
            If there is already a visualization attached with the given name, it is quietly replaced.
        */
        virtual void attachVisualization(const std::string& name, VisualizationNodePtr v);

        /*!
            Remove an attached visualization.
        */
        virtual void detachVisualization(const std::string& name);

        /*!
            Setup the visualization of this object.
            \param showVisualization If false, the visualization is disabled.
            \param showAttachedVisualizations If false, the visualization of any attached optional visualizations is disabled.
        */
        virtual void setupVisualization(bool showVisualization, bool showAttachedVisualizations);


        /*!
                Clone this visualization.
                \param deepCopy When true, the underlying visualization is copied, otherwise a reference to the existing visualization is passed.
                \param scaling Scale Can be set to create a scaled version of this visual data.
                Since the underlying implementation may be able to re-use the visualization data, a deep copy may not be necessary in some cases.
            */
        virtual VisualizationNodePtr clone(bool deepCopy = true, float scaling = 1.0f);

        virtual std::string getType()
        {
            return CoinVisualizationFactory::getName();
        }

        /*!
            Saves model file to model path. By default VRML models are generated.
            \param modelPath The directory.
            \param filename The new filename. If filename extension is ".iv", the file is stored in Open Inventor format. Otherwise the file is stored in VRML2 format (.wrl).
        */
        virtual bool saveModel(const std::string& modelPath, const std::string& filename);

        virtual void createTriMeshModel();

    protected:


        /*!
            Replace current visualization of this node.
            Be careful: any former grabbed trimeshmodels do no longer represent the new datastructure!
        */
        void setVisualization(SoNode* newVisu);

        SoNode* visualization;
        SoSeparator* visualizationAtGlobalPose;
        SoSeparator* attachedVisualizationsSeparator;
        SoSeparator* scaledVisualization;
        std::map< std::string, SoNode* > attachedCoinVisualizations;    //< These optional visualizations will not show up in the TriMeshModel

        SoMatrixTransform* globalPoseTransform;
        TriMeshModelPtr triMeshModel;
        SoScale* scaling;

        static void InventorTriangleCB(void* data, SoCallbackAction* action,
                                       const SoPrimitiveVertex* v1,
                                       const SoPrimitiveVertex* v2,
                                       const SoPrimitiveVertex* v3);
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_CoinVisualizationNode_h_
