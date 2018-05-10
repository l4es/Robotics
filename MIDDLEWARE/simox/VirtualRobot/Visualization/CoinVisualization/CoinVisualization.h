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
#ifndef _VirtualRobot_CoinVisualization_h_
#define _VirtualRobot_CoinVisualization_h_

#include "../../VirtualRobotImportExport.h"
#include "../Visualization.h"

#include <Inventor/nodes/SoSelection.h>

class SoNode;

namespace VirtualRobot
{

    /*!
        A Coin3D based implementation of a visualization.
    */
    class VIRTUAL_ROBOT_IMPORT_EXPORT CoinVisualization : public Visualization
    {
    public:
        //EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        CoinVisualization(const VisualizationNodePtr visualizationNode);
        CoinVisualization(const std::vector<VisualizationNodePtr>& visualizationNodes);
        virtual ~CoinVisualization();

        /*!
            To see the visualizations in an SoExaminerViewer enable an highlight render action
            e.g. viewer->setGLRenderAction(new SoLineHighlightRenderAction);
        */
        virtual bool highlight(VisualizationNodePtr visualizationNode, bool enable);
        virtual bool highlight(unsigned int which, bool enable);
        virtual bool highlight(bool enable);
        virtual bool highlight(SoNode* visu, bool enable);

        virtual VisualizationPtr clone();

        SoNode* getCoinVisualization();

        void exportToVRML2(std::string filename);

        static std::string getFactoryName()
        {
            return "inventor";
        }

    protected:
        bool buildVisualization();
        SoSelection* selection;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_CoinVisualization_h_
