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

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "ImgDiffs"

#include "imgdiffs.h"

namespace RDK2 { namespace RGraphics {

void compressImgLine(RImage::Type type, uint8_t** rows, int y, int width, vector<uint8_t>& cLine)
{
	switch (type) {
		case RImage::C8: {
		#if RLE_MODE == 2
			int x = 0;
			while (x < width) {
				uint8_t color = rows[y][x];
				uint8_t count = 0;
				do { count++; x++; } while (x < width && color == rows[y][x] && count < 255);
				cLine.push_back(color);
				cLine.push_back(count-1);	// 0 means 1
			}
		#elif RLE_MODE == 1
			int x = 0;
			while (x < width) {
				uint8_t color = rows[y][x];
				uint8_t count = 0;
				do { count++; x++; } while (x < width && color == rows[y][x] && count < 31);
				cLine.push_back(((reduceC8(color)&7)|((count-1)<<3)));	// count = 0 means 1
			}
		#else
			for (int x = 0; x < width; x++) cLine.push_back(rows[y][x]);
		#endif
			
		} break;

		case RImage::RGB32: {
			int x = 0;
			while (x < width) {
				int xx = x*RImage::getBytesPerPixel(type);
				cLine.push_back(rows[y][xx]);
				cLine.push_back(rows[y][xx+1]);
				cLine.push_back(rows[y][xx+2]);
				x++;
			}
		} break;

		default: {
			int x = 0;
			while (x < width) {
				int xx = x * RImage::getBytesPerPixel(type);
				for (size_t i = 0; i < RImage::getBytesPerPixel(type); i++) {
					cLine.push_back(rows[y][xx+i]);
				}
				x++;
			}
		}
	}
}

void uncompressImgDiffCBuf(const vector<uint8_t>& cBuf, RImageDiffRect* diff)
{
	int x = 0, y = 0, diffCursor = 0;
	uint8_t** px = diff->getPixels();
	switch (diff->refImageType) {
		case RImage::C8: {
		#if RLE_MODE == 2
			while (y < diff->rectHeight) {
				uint8_t color = cBuf[diffCursor++];
				uint8_t count = cBuf[diffCursor++];
				for (size_t i = 0; i < (uint) count+1; i++) {
					if (y >= diff->rectHeight) {
						RDK_ERROR_PRINTF("Diff buffer gone past image end");
						break;
					}
					px[y][x] = color;
					if (++x >= diff->rectWidth) { x = 0; y++; }
				}
			}
		#elif RLE_MODE == 1
			while (y < diff->rectHeight) {
				uint8_t color = expandC8((cBuf[diffCursor])&7);
				uint8_t count = (cBuf[diffCursor++])>>3;
				for (size_t i = 0; i < (uint) count+1; i++) {
					if (y >= diff->rectHeight) {
						RDK_ERROR_PRINTF("Diff buffer gone past image end");
						break;
					}
					px[y][x] = color;
					if (++x >= diff->rectWidth) { x = 0; y++; }
				}
			}
		#else
			while (y < diff->rectHeight) {
				px[y][x] = cBuf[diffCursor++];
				if (++x >= diff->rectWidth) { x = 0; y++; }
			}
		#endif
		} break;

		case RImage::RGB32: {
			while (y < diff->rectHeight) {
				int xx = x*RImage::getBytesPerPixel(diff->refImageType);
				px[y][xx] = cBuf[diffCursor++];
				px[y][xx+1] = cBuf[diffCursor++];
				px[y][xx+2] = cBuf[diffCursor++];
				if (++x >= diff->rectWidth) { x = 0; y++; }
			}
		} break;

		default: {
			while (y < diff->rectHeight) {
				px[y][x] = cBuf[diffCursor++];
				if (++x >= diff->rectWidth) { x = 0; y++; }
			}
		}
	}
}

}} // namespaces
