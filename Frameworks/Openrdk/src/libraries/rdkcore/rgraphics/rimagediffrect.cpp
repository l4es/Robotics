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
#define LOGGING_MODULE "RImageDiffRect"

#include "rimagediffrect.h"
#include "imgdiffs.h"

namespace RDK2 { namespace RGraphics {

	RDK2_FACTORY(RImageDiffRect);

RImageDiffRect::RImageDiffRect(
	RImage* image, int left, int top, int width, int height) 
:
	ObjectDiff("RImage"), rectLeft(left), rectTop(top), rectWidth(width), 
	rectHeight(height), refImageWidth(image->getWidth()), 
	refImageHeight(image->getHeight()), refImageType(image->getType()),
	buffer(0), rows(0)
{
	if (left + width > (int)image->getWidth()) 
		rectWidth = image->getWidth() - left;
	if (top + height > (int)image->getHeight()) 
		rectHeight = image->getHeight() - top;
	if (rectWidth < 0 || rectHeight < 0) 
		return;	// FIXME occhio, buffer non inizializzato; eccezione?
	initBuffer(rectWidth, rectHeight, image->getType());
	copyRect(image->getBuffer(), top, left, rectWidth, rectHeight, image->getType());
}

RImageDiffRect::RImageDiffRect(
	RImageDiffRect* imageDiffRect, int left, int top, int width, int height) 
:
	ObjectDiff("RImage"), rectLeft(left), rectTop(top), rectWidth(width),
	rectHeight(height),
	refImageWidth(imageDiffRect->refImageWidth), 
	refImageHeight(imageDiffRect->refImageHeight),
	refImageType(imageDiffRect->refImageType), buffer(0), rows(0)
{
	if (left + width > imageDiffRect->rectWidth) 
		rectWidth = imageDiffRect->rectWidth - left;
	if (top + height > imageDiffRect->rectHeight) 
		rectHeight = imageDiffRect->rectHeight - top;
	if (rectWidth < 0 || rectHeight < 0) 
		return;	// FIXME occhio, buffer non inizializzato; eccezione?
	initBuffer(rectWidth, rectHeight, imageDiffRect->refImageType);
	uint8_t* px = imageDiffRect->buffer;
	copyRect(px, top, left, rectWidth, rectHeight, imageDiffRect->refImageType);
}

RImageDiffRect::RImageDiffRect() :
	ObjectDiff("RImage"), rectLeft(0), rectTop(0), rectWidth(0), rectHeight(0),
	refImageWidth(0), refImageHeight(0), refImageType(RImage::C8),
	buffer(0), rows(0)
{ }

uint8_t** RImageDiffRect::getPixels()
{
	cBuf.clear();	// if someone modify the pixels, cBuf becomes invalid
	return rows;
}

uint8_t* RImageDiffRect::getBuffer()
{
	cBuf.clear();	// if someone modify the pixels, cBuf becomes invalid
	return buffer;
}

RDK2::Object* RImageDiffRect::clone() const
{
	RImageDiffRect* newDiff = new RImageDiffRect(*this);
	return newDiff;	// FIXME
	newDiff->buffer = 0;	// this avoids initBuffer to delete MY buffer
	newDiff->rows = 0;	// this avoids initBuffer to delete MY buffer
	newDiff->initBuffer(newDiff->rectWidth, newDiff->rectHeight, newDiff->refImageType);
	newDiff->copyRect(this->buffer, 0, 0, rectWidth, rectHeight, newDiff->refImageType);
	return newDiff;
}

void RImageDiffRect::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
		objectClassName = r->readString();
		rectLeft = r->read_i32();
		rectTop = r->read_i32();
		rectWidth = r->read_i32();
		rectHeight = r->read_i32();
		refImageWidth = r->read_i32();
		refImageHeight = r->read_i32();
		refImageType = (RImage::Type) r->read_i32();
		initBuffer(rectWidth, rectHeight, refImageType);
		uint8_t* readBuffer;
		size_t readLen;
		r->readBytes((void**)&readBuffer, &readLen);
		cBuf.assign(readBuffer, readBuffer + readLen);
		uncompressImgDiffCBuf(cBuf, this);
	r->doneReading();
}

void RImageDiffRect::write(Writer* w) const throw (WritingException)
{
	w->startWriting(getClassName());
		w->writeString(objectClassName);
		w->write_i32(rectLeft);
		w->write_i32(rectTop);
		w->write_i32(rectWidth);
		w->write_i32(rectHeight);
		w->write_i32(refImageWidth);
		w->write_i32(refImageHeight);
		w->write_i32(refImageType);
		if (cBuf.empty()) {
			RDK_DEBUG_PRINTF("Compressed buffer is empty, (re)compressing");
			for (int y = 0; y < rectHeight; y++) {
				vector<uint8_t> cLine;
				compressImgLine(refImageType, rows, y, rectWidth, cLine);
				cBuf.insert(cBuf.end(), cLine.begin(), cLine.end());
			}
		}
		w->writeBytes(&(cBuf[0]), cBuf.size());
	w->doneWriting();
}

// FIXME assumo che, per qualunque immagine, almeno una riga entra dentro un buffer di lunghezza maxBufferSize
// se questo non dovesse essere vero, viene creato comunque un diff con buffer contenente una riga
vector<ObjectDiff*> RImageDiffRect::split(size_t maxBufferSize)
{
	vector<ObjectDiff*> ret;
	vector<uint8_t> cLine;
	vector<uint8_t> cBuf;
	int startY = 0;
	for (int y = 0; y < rectHeight; y++) {
		compressImgLine(refImageType, rows, y, rectWidth, cLine);
		if (cBuf.size() + cLine.size() > maxBufferSize) {
			RImageDiffRect* newDiff = new RImageDiffRect(this, 0, startY, rectWidth, y - startY);
			newDiff->cBuf.assign(cBuf.begin(), cBuf.end());
			if (cBuf.size() > 0) ret.push_back(newDiff);
			startY = y;
			cBuf.clear();
		}
		cBuf.insert(cBuf.end(), cLine.begin(), cLine.end());
		cLine.clear();
	}
	if (cBuf.size() > 0) {
		RImageDiffRect* newDiff = new RImageDiffRect(this, 0, startY, rectWidth, rectHeight - startY);
		newDiff->cBuf.assign(cBuf.begin(), cBuf.end());
		ret.push_back(newDiff);
	}
	return ret;
}

RImageDiffRect::~RImageDiffRect()
{
	delete[] buffer;
	delete[] rows;
}

void RImageDiffRect::initBuffer(int width, int height, RImage::Type type)
{
	rectWidth = width;
	rectHeight = height;
	refImageType = type;
	
	int pixsize=RImage::getBytesPerPixel(type);
	int size=width*height*pixsize;
	int sizerow=width*pixsize;

	delete[] buffer;
	delete[] rows;

	buffer=new uint8_t[size];
	rows=new uint8_t*[height];
	
	for (int i=0;i<height;i++) rows[i]=buffer+sizerow*i;
	cBuf.clear();
}

void RImageDiffRect::copyRect(unsigned char* /*buf*/, int /*top*/, int /*left*/, int /*width*/, int /*height*/, RImage::Type /*type*/)
{
	RDK_ERROR_PRINTF("This function is not implemented");
/*	int pixsize = RImage::getBytesPerPixel(type);
	for (int y = 0; y < height; y++) {
		for (int i = 0; i < width * pixsize; i++)
			//this->rows[y][i] = buf[(y+top)*	// rows[y+top][i+left*pixsize];
	}*/
}

}} // namespaces
