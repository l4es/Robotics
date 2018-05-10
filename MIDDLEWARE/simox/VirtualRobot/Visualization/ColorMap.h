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
* @author     David Gonzales, Nikolaus Vahrenkamp
* @copyright  2011 David Gonzales, Nikolaus Vahrenkamp
*             GNU Lesser General Public License
*
*/
#ifndef _VirtualRobot_ColorMap_h_
#define _VirtualRobot_ColorMap_h_

#include "../VirtualRobotImportExport.h"
#include "VisualizationFactory.h"

#include <string>

namespace VirtualRobot
{

    class VIRTUAL_ROBOT_IMPORT_EXPORT ColorMap
    {
    public:

        enum type
        {
            eIntensity, eHot, eRed, eGreen, eBlue, eHotAlpha, eRedAlpha, eBlueAlpha, eGreenAlpha
        };
        ColorMap(type t);

        virtual ~ColorMap();

        /*!
            Returns color that is equivalent to position.
            \param position A value between 0 and 1.
            \return The corresponding color values (r,g,b,transparency values are in [0..1])
        */
        VirtualRobot::VisualizationFactory::Color getColor(float position) const;
        bool getColor(float position, VirtualRobot::VisualizationFactory::Color& storeColor) const;

        //! Custom color maps can be created with this method.
        static ColorMap customColorMap(std::vector< VirtualRobot::VisualizationFactory::Color > colors);

    protected:
        ColorMap();

        bool addColorKey(const unsigned char R, const unsigned char G, const unsigned char B, const unsigned char A, const float Position);

        void create(type t);
        struct ColorKey
        {
            unsigned int index;
            VisualizationFactory::Color color; // internally alpha values are stored in the transparency variable
            float position;
        };
        void sort();
        static bool CompareColorKey(const ColorKey& lhs, const ColorKey& rhs);

        std::vector<ColorKey> colorKeys;
        std::vector<float> intervals;

        type colorMapType;

    };

} // namespace VirtualRobot

#endif // _VirtualRobot_ColorMap_h_
