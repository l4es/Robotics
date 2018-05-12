/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi (<first_name>.<last_name>@dis.uniroma1.it)
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

#ifndef RDKCORE_INTEROP_MAGICKPP
#define RDKCORE_INTEROP_MAGICKPP

#include <rdkcore/config.h>

#ifdef MagickPP_FOUND
#include <Magick++.h>
#endif

#include <rdkcore/rgraphics/rimage.h>

namespace RDK2 {

using namespace RDK2::RGraphics;

#ifdef MagickPP_FOUND
Magick::Image convertToMagickImage(const RImage* rimg);
RImage* convertToRImage(const Magick::Image& mimg);
#endif

}

#endif
