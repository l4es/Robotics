/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef RDK2_RGRAPHICS_COLOR
#define RDK2_RGRAPHICS_COLOR

#include <map>
#include <stdint.h>
#include <string>

namespace RDK2 { namespace RGraphics {

typedef uint32_t RgbColor;

inline unsigned char rgbGetRed(RgbColor   c) { return (unsigned char) ((c & 0xff0000) >> 16); }
inline unsigned char rgbGetGreen(RgbColor c) { return (unsigned char) ((c & 0x00ff00) >> 8); }
inline unsigned char rgbGetBlue(RgbColor  c) { return (unsigned char) ((c & 0x0000ff)); }

std::string getColorName(RgbColor color);
RgbColor getColorByName(const std::string& name);

namespace Meta
{
	struct ColorToNameReference
	{
		ColorToNameReference(RgbColor color, std::string name);
	};

	struct NameToColorReference
	{
		NameToColorReference(std::string name, RgbColor color);
	};

	typedef std::map<RgbColor,std::string> ColorToNameReferenceTable;
	typedef std::map<std::string,RgbColor> NameToColorReferenceTable;

	ColorToNameReferenceTable& getColorToNameReferenceTable();
	NameToColorReferenceTable& getNameToColorReferenceTable();
}

#define DECLARE_COLOR(n,v) \
	const RgbColor RGB_##n = v; \
	const RDK2::RGraphics::Meta::ColorToNameReference __RGB_color2name_##n(RGB_##n,#n); \
	const RDK2::RGraphics::Meta::NameToColorReference __RGB_name2color_##n(#n,RGB_##n);

DECLARE_COLOR(BLACK     , 0x00000000);
DECLARE_COLOR(WHITE     , 0x00ffffff);
DECLARE_COLOR(RED       , 0x00ff0000);
DECLARE_COLOR(GREEN     , 0x0000ff00);
DECLARE_COLOR(BLUE      , 0x000000ff);
DECLARE_COLOR(YELLOW    , 0x00ffff00);
DECLARE_COLOR(MAGENTA   , 0x00ff00ff);
DECLARE_COLOR(CYAN      , 0x0000ffff);
DECLARE_COLOR(GRAY      , 0x00bebebe);
DECLARE_COLOR(ORANGE    , 0x00ffa500);
DECLARE_COLOR(PINK      , 0x00ffc0cb);
DECLARE_COLOR(VIOLET    , 0x00ee82ee);
DECLARE_COLOR(GOLD      , 0x00ffd700);
DECLARE_COLOR(LIME_GREEN, 0x007cfc00);
DECLARE_COLOR(LAWN_GREEN, 0x0032cd32);

}} // namespace

#endif

