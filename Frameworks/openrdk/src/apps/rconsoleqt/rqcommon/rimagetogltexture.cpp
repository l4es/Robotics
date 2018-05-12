/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi (daniele.calisi@dis.uniroma1.it)
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

#include "rimagetogltexture.h"
#include "rqcommon.h"

#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RImageToGlTexture"

#include <cmath>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RGraphics;

const char *RImageToGlTexture::glErrorToString(GLenum error)
{
	#define RETURN_ERROR(errname) case errname: return #errname;
	switch (error) {
		RETURN_ERROR(GL_INVALID_ENUM)
		RETURN_ERROR(GL_INVALID_VALUE)
		RETURN_ERROR(GL_INVALID_OPERATION)
		RETURN_ERROR(GL_STACK_OVERFLOW)
		RETURN_ERROR(GL_STACK_UNDERFLOW)
		RETURN_ERROR(GL_OUT_OF_MEMORY)
		default: return "Unknown error";
	}
}

bool RImageToGlTexture::printGlErrors(const char *when)
{
	bool retval = false;
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) {
		retval = true;
		RDK_ERROR_PRINTF("%s raised the following GL error(s):", when);
		while (err != GL_NO_ERROR) {
			RDK_ERROR_PRINTF(glErrorToString(err));
			err = glGetError();
		}
	}
	return retval;
}

RImageToGlTexture::RImageToGlTexture() : haveToUpdateWholeMap(true)
{
	bool exiting = false;
	QT_THREAD_GUARD()
	memset(C8ToGlR, 0, 256 * sizeof(GLfloat));
	memset(C8ToGlG, 0, 256 * sizeof(GLfloat));
	memset(C8ToGlB, 0, 256 * sizeof(GLfloat));
	
	C8ToGlR[RImage::C8Black] = 0.0;   C8ToGlG[RImage::C8Black] = 0.0;   C8ToGlB[RImage::C8Black] = 0.0;
	C8ToGlR[RImage::C8White] = 1.0;   C8ToGlG[RImage::C8White] = 1.0;   C8ToGlB[RImage::C8White] = 1.0;
	C8ToGlR[RImage::C8Blue] = 0.0;    C8ToGlG[RImage::C8Blue] = 0.0;    C8ToGlB[RImage::C8Blue] = 1.0;
	C8ToGlR[RImage::C8Red] = 1.0;     C8ToGlG[RImage::C8Red] = 0.0;     C8ToGlB[RImage::C8Red] = 0.0;
	C8ToGlR[RImage::C8Green] = 0.0;   C8ToGlG[RImage::C8Green] = 1.0;   C8ToGlB[RImage::C8Green] = 0.0;
	C8ToGlR[RImage::C8Cyan] = 0.0;    C8ToGlG[RImage::C8Cyan] = 1.0;    C8ToGlB[RImage::C8Cyan] = 1.0;
	C8ToGlR[RImage::C8Magenta] = 1.0; C8ToGlG[RImage::C8Magenta] = 0.0; C8ToGlB[RImage::C8Magenta] = 1.0;
	C8ToGlR[RImage::C8Grey] = 0.5;    C8ToGlG[RImage::C8Grey] = 0.5;    C8ToGlB[RImage::C8Grey] = 0.5;
	
	lastImageWidth = 0;
	lastImageHeight = 0; 
	lastImageType = RImage::C8;

	glClearColor(0.0, 0.0, 0.0, 1.0);
	
	glPixelTransferi(GL_MAP_COLOR, GL_TRUE);
	glPixelMapfv(GL_PIXEL_MAP_I_TO_R, 256, C8ToGlR);
	glPixelMapfv(GL_PIXEL_MAP_I_TO_G, 256, C8ToGlG);
	glPixelMapfv(GL_PIXEL_MAP_I_TO_B, 256, C8ToGlB);
	
	glDisable(GL_LIGHTING);
	
	glEnable(GL_TEXTURE_2D);
	
	glGenTextures(1, &textureId);
	
	glBindTexture(GL_TEXTURE_2D, textureId);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	
	printGlErrors("initialize():");
}

RImageToGlTexture::~RImageToGlTexture()
{
	glDeleteTextures(1, &textureId);
}

void RImageToGlTexture::updateTexture(RImage* rimg)
{
	isMap = false;
	update_texture(rimg, 0, 0, rimg->getWidth(), rimg->getHeight());
}

void RImageToGlTexture::updateTexture(RImage* rimg, int x, int y, int w, int h)
{
	isMap = false;
	update_texture(rimg, x, y, w, h);
}

void RImageToGlTexture::updateTexture(RMapImage* rmi)
{
	isMap = true;
	lastMapWidth = rmi->getRealWidth();
	lastMapX = rmi->x;
	lastMapY = rmi->y;
	lastMapTheta = rmi->theta;
	update_texture(rmi->image, 0, 0, rmi->image->getWidth(), rmi->image->getHeight());
}

void RImageToGlTexture::updateTexture(RMapImage* rmi, int x, int y, int w, int h)
{
	isMap = true;
	lastMapWidth = rmi->getRealWidth();
	lastMapX = rmi->x;
	lastMapY = rmi->y;
	lastMapTheta = rmi->theta;
	update_texture(rmi->image, x, y, w, h);
}

void RImageToGlTexture::update_texture(RImage* rimg, int x, int y, int w, int h)
{
	bool exiting = false;
	QT_THREAD_GUARD()
	glBindTexture(GL_TEXTURE_2D, getTextureId());
	printGlErrors("updateTexture: glBindTexture");
	
	if (haveToUpdateWholeMap) {
		haveToUpdateWholeMap = false;
		x = 0; y = 0; w = rimg->getWidth(); h = rimg->getHeight();
	}
	else if ((w == 0) || (h == 0)) {
		//RDK_ERROR_PRINTF("updateTexture called with empty zone");
		//return;
		//x = 0; y = 0; w = rimg->getWidth(); h = rimg->getHeight();
		return;
	}
	
	if (rimg->getWidth() != (size_t) lastImageWidth || rimg->getHeight() != (size_t) lastImageHeight) {
		// the image has changed (or it has been resized) or
		// there has been a full update request
		size_t e = 0;
		// texWidth and texHeight need to be exact powers of 2
		while ((size_t) (1 << e) < rimg->getWidth()) e++;
		texWidth = (unsigned int) (1 << e);
		
		e = 0;
		while ((size_t) (1 << e) < rimg->getHeight()) e++;
		texHeight = (unsigned int) (1 << e);
		// texWidth and texHeight need to be equal
		if (texHeight > texWidth) texWidth = texHeight;
		else texHeight = texWidth;
		
		// Safety check
		GLint maxTextureSize;
		glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
		
		if (texHeight > (unsigned int)maxTextureSize) {
			// A possible fix for this problem was found here:
			// https://bugs.launchpad.net/ubuntu/+source/xserver-xorg-video-ati/+bug/21893
			RDK_ERROR_PRINTF("ERROR! I should create a texture");
			RDK_ERROR_PRINTF("%dx%d pixels wide, but the",
					texWidth, texHeight);
			RDK_ERROR_PRINTF("maximum texture size supported by");
			RDK_ERROR_PRINTF("your driver is %dx%d!",
					maxTextureSize, maxTextureSize);
			RDK_ERROR_PRINTF("Strange things will happen now.");
			RDK_ERROR_PRINTF("Try adding the line: ");
			RDK_ERROR_PRINTF("<option name=\"allow_large_textures\" value=\"2\" />");
			RDK_ERROR_PRINTF("Inside the file /etc/drirc");
		}

		size_t initialBufferSize = 0;
		if (rimg->getType() != RImage::C8 && rimg->getType() != RImage::GREY && rimg->getType() != RImage::RGB32
		&& rimg->getType() != RImage::RGB24) initialBufferSize = texWidth * texHeight * RImage::getBytesPerPixel(RImage::RGB32);
		else initialBufferSize = texWidth * texHeight * rimg->getBytesPerPixel();
		unsigned char* bytes = new unsigned char[initialBufferSize];
		int defcol = 128;
		if (rimg->getType() == RImage::C8) defcol = 2;
		memset(bytes, defcol, initialBufferSize);
		
		if (rimg->getType() == RImage::C8) glPixelTransferi(GL_MAP_COLOR, GL_TRUE);
		else glPixelTransferi(GL_MAP_COLOR, GL_FALSE);
		printGlErrors("updateTexture: glPixelTransferi");
		
		if (rimg->getType() == RImage::C8) {
			glTexImage2D(GL_TEXTURE_2D, 0, 3, texWidth, texHeight, 0, GL_COLOR_INDEX, GL_UNSIGNED_BYTE, bytes);
		}
		else if (rimg->getType() == RImage::GREY) {
			glTexImage2D(GL_TEXTURE_2D, 0, 1, texWidth, texHeight, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, bytes);
		}
		else if (rimg->getType() == RImage::RGB32) {
			glTexImage2D(GL_TEXTURE_2D, 0, 3, texWidth, texHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, bytes);
		}
		else if (rimg->getType() == RImage::RGB24) {
			glTexImage2D(GL_TEXTURE_2D, 0, 3, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, bytes);
		}
		else {
			glTexImage2D(GL_TEXTURE_2D, 0, 3, texWidth, texHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, bytes);
		}

		// These GL errors may need special messages
		int err = glGetError();
		if (err != GL_NO_ERROR) {
			if (err == GL_INVALID_OPERATION) {
				RDK_ERROR_PRINTF("It seems that your OpenGL implementation does"
				" not support COLOR_INDEX textures, the recommended (not always working)"
				" libGL.so version is 1.3 or 1.4. The map will be not showed.");
				RDK_DEBUG_PRINTF("(Current OpenGL version: %s)", glGetString(GL_VERSION));
			}
			else  {
				RDK_ERROR_PRINTF("There are one or more OpenGL errors:");
				// glGetError will return errors until
				// we got them all
				while (err != GL_NO_ERROR) {
					RDK_ERROR_PRINTF(glErrorToString(err));
					err = glGetError();
				}
				RDK_ERROR_PRINTF("The image won't be shown.");
			}
		}
		
		delete[] bytes;
		
		lastImageWidth = rimg->getWidth();
		lastImageHeight = rimg->getHeight();
	}
	
	if (rimg->getType() == RImage::C8) {
		glPixelTransferi(GL_MAP_COLOR, GL_TRUE);
		printGlErrors("updateTexture: glPixelTransferi (C8)");

/*		glPixelStorei(GL_PACK_ROW_LENGTH, 1);
		glPixelStorei(GL_PACK_ALIGNMENT, 1);*/
		// XXX dovrebbero farlo le righe qui sopra... ma GL_PACK_ROW_LENGTH non funziona
		// quindi mi ricopio il pezzo di mappa in un buffer e passo quello a glTexSubImage2D
		unsigned char* imgbuf = new unsigned char[w * h];
		size_t i = 0;
		for (int ay = y; ay < h + y; ay++) {
			for (int ax = x; ax < w + x; ax++) {
				if ((int)i < (w * h) && ax < (int)rimg->getWidth() && ay < (int)rimg->getHeight())
					imgbuf[i++] = rimg->getBuffer()[ay * rimg->getWidth() + ax];	// FIXME (if???)
			}
		}
		glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h, 
			GL_COLOR_INDEX, GL_UNSIGNED_BYTE, imgbuf
			/*rimg->getBuffer() + y * rimg->getWidth() + x*/);
		delete[] imgbuf;
	}
	else if (rimg->getType() == RImage::GREY) {
		glPixelTransferi(GL_MAP_COLOR, GL_FALSE);
		printGlErrors("updateTexture: glPixelTransferi (GREY)");
		glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h, GL_LUMINANCE, GL_UNSIGNED_BYTE, rimg->getBuffer());
	}
	else if (rimg->getType() == RImage::RGB32) {
		glPixelTransferi(GL_MAP_COLOR, GL_FALSE);
		printGlErrors("updateTexture: glPixelTransferi (RGB32)");
		glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rimg->getBuffer());
	}
	else if (rimg->getType() == RImage::RGB24) {
		glPixelTransferi(GL_MAP_COLOR, GL_FALSE);
		printGlErrors("updateTexture, glPixelTransferi (RGB24)");
		glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h, GL_RGB, GL_UNSIGNED_BYTE, rimg->getBuffer());
	}
	else {
		RImage* rgbimg = rimg->convertTo(RImage::RGB32);
		glPixelTransferi(GL_MAP_COLOR, GL_FALSE);
		printGlErrors("updateTexture: glPixelTransferi (Other)");
		glTexSubImage2D(GL_TEXTURE_2D, 0, x, y, w, h, GL_RGBA, GL_UNSIGNED_BYTE, rgbimg->getBuffer());
		delete rgbimg;
	}
	printGlErrors("updateTexture: glTexSubImage2D");
}

void RImageToGlTexture::drawMapQuad()
{
	bool exiting = false;
	QT_THREAD_GUARD()
	glBindTexture(GL_TEXTURE_2D, getTextureId());

	double texcoordx = (float) lastImageWidth / getTextureWidth();
	double texcoordy = (float) lastImageHeight / getTextureHeight();

	double lastMapHeight = lastMapWidth * lastImageHeight / lastImageWidth;
	
	glBegin(GL_QUADS);
	if (isMap) {
		glTexCoord2f(0, 0);
		glVertex3f(lastMapX, lastMapY, 0.0);

		glTexCoord2f(texcoordx, 0);
		glVertex3f(lastMapX + lastMapWidth, lastMapY, 0.0);

		glTexCoord2f(texcoordx, texcoordy);
		glVertex3f(lastMapX + lastMapWidth, lastMapY - lastMapHeight, 0.0);

		glTexCoord2f(0, texcoordy);
		glVertex3f(lastMapX, lastMapY - lastMapHeight, 0.0);
	}
	else {
		glTexCoord2f(0, 0);
		glVertex3f(0, 0, 0.0);
		
		glTexCoord2f(texcoordx, 0);
		glVertex3f(lastImageWidth, 0, 0.0);

		glTexCoord2f(texcoordx, texcoordy);
		glVertex3f(lastImageWidth, -lastImageHeight, 0.0);

		glTexCoord2f(0, texcoordy);
		glVertex3f(0, -lastImageHeight, 0.0);
	}
	glEnd();
}

}} // namespaces
