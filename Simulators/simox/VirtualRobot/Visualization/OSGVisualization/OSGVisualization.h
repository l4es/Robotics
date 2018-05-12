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
* @author     Nikolaus Vahrenkamp
* @copyright  2011 Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_OSGVisualization_h_
#define _VirtualRobot_OSGVisualization_h_

#include "../../VirtualRobotImportExport.h"
#include "../Visualization.h"

#include <osg/Node>
#include <osg/Group>
namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT OSGVisualization : public Visualization
    {
    public:

        OSGVisualization(const VisualizationNodePtr visualizationNode);
        OSGVisualization(const std::vector<VisualizationNodePtr>& visualizationNodes);
        virtual ~OSGVisualization();

        virtual bool highlight(VisualizationNodePtr visualizationNode, bool enable);
        virtual bool highlight(unsigned int which, bool enable);
        virtual bool highlight(bool enable);
        virtual bool highlight(osg::Node* visu, bool enable);

        virtual VisualizationPtr clone();

        osg::Node* getOSGVisualization();

        static std::string getFactoryName()
        {
            return "osg";
        }

        //virtual void setGlobalPose(const Eigen::Matrix4f &p);

    protected:
        bool buildVisualization();
        osg::Group* visu;
    };

} // namespace VirtualRobot

#endif // _VirtualRobot_OSGVisualization_h_
