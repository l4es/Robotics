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

#ifndef RDK2_RGRAPHICS_IMGDIFFS
#define RDK2_RGRAPHICS_IMGDIFFS

#include "rimage.h"
#include "rimagediffrect.h"

namespace RDK2 { namespace RGraphics {

// for RImage::C8 image type:
// 1 means 1-byte RLE encoding
// 2 means 2-bytes RLE encoding
// other (0) means no RLE encoding
#define RLE_MODE 1

void compressImgLine(RImage::Type type, uint8_t** rows, int y, int width, vector<uint8_t>& cLine);
void uncompressImgDiffCBuf(const vector<uint8_t>& cBuf, RImageDiffRect* diff);

}} // namespaces

#endif
