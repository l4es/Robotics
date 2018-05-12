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

#include <ctype.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <string>
#include <list>
using namespace std;

#include <rdkcore/textutils/textutils.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RImage"

#include "rimagediffrect.h"
#include "rimagediffpoints.h"
#include "rimagediffjpegpacket.h"
#include "rimage.h"
#include "imgdiffs.h"
//#include "jpegstuff/sendPicture.h"

namespace RDK2 { namespace RGraphics {

#define JPEG_QUALITY 25
#undef RIMAGE_RGB32_JPEG_SENDING

vector<ObjectDiff*> RImage::splitInDiffs(size_t maxBufferSize)
{
#if RIMAGE_RGB32_JPEG_SENDING
	if (type == RGB32) {
		SendPicture sp(buffer);
	  	sp.setsize(getWidth(), getHeight());

		LList<TransmissionUnit>* transmissionlist = sp.getZZ(JPEG_QUALITY, 1, 1);
		
		vector<ObjectDiff*> ret;
		LList <TransmissionUnit>::entry* trans_e;
		
		uint pnum = 0;
		for (trans_e = transmissionlist->first(); trans_e->next; trans_e = trans_e->next) {
		  	TransmissionUnit *transUnit = &(trans_e->x);
		  	RImageDiffJpegPacket* ridp = new RImageDiffJpegPacket(this);
		  	ridp->packetNumber = pnum++;
		  	for (int i = 0; i < transUnit->k; i++) {
		  		ridp->data.push_back(transUnit->data[i]);
  			}
  			ret.push_back(ridp);
		}
		
		return ret;
	}
	else {
		RImageDiffRect* bigDiff = new RImageDiffRect(this, 0, 0, getWidth(), getHeight());
		vector<ObjectDiff*> ret = bigDiff->split(maxBufferSize);
		delete bigDiff;
		return ret;
	}
#else
	RImageDiffRect* bigDiff = new RImageDiffRect(this, 0, 0, getWidth(), getHeight());
	vector<ObjectDiff*> ret = bigDiff->split(maxBufferSize);
	delete bigDiff;
	return ret;
#endif
}

bool RImage::applyDiff(const ObjectDiff* diff)
{
	const RImageDiffRect* rectDiff = dynamic_cast<const RImageDiffRect*>(diff);
	if (rectDiff) {
		RDK_ERROR_PRINTF("Diff applying of RImage diff rect is not implemented");
/*		setSizeAndType(rectDiff->refImageWidth, rectDiff->refImageHeight, rectDiff->refImageType);
		uint8_t** rpx = rectDiff->rows;
		for (int y = 0; y < rectDiff->rectHeight; y++) {
			for (int x = 0; x < rectDiff->rectWidth; x++) {
				for (int i = 0; i < getBytesPerPixel(type); i++) {
					rows[y+rectDiff->rectTop][(x+rectDiff->rectLeft)*getPixelSize()+i] =
						rpx[y][x*getPixelSize()+i];
				}
			}
		}

		return true;*/
		return false;
	}
	const RImageDiffPoints* pointsDiff = dynamic_cast<const RImageDiffPoints*>(diff);
	if (pointsDiff) {
		setSizeAndType(rectDiff->refImageWidth, rectDiff->refImageHeight, rectDiff->refImageType);
		if (getType() == RImage::C8) {
			for (size_t i = 0; i < pointsDiff->points.size(); i++) {
				buffer[pointsDiff->points[i].y * this->width + pointsDiff->points[i].x] = pointsDiff->colorsC8[i];
			}
		}
		return true;
	}
/*	const RImageDiffJpegPacket* jpegPacket = dynamic_cast<const RImageDiffJpegPacket*>(diff);
	if (jpegPacket) {
		if (jpegPacket->id == jpegCache.currentId) {
			jpegCache.pushPacket(jpegPacket);
		}
		else if (jpegPacket->id > jpegCache.currentId) {
			jpegCache.refreshImage(getBuffer(), getWidth(), getHeight());
			jpegCache.clearPackets();
			jpegCache.pushPacket(jpegPacket);
		}
		// otherwise, it is an old packet: drop it
	}*/
	return false;
}

}} // namespaces
