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

#ifndef RDK2_RQM_RIMAGETOGLTEXTURE
#define RDK2_RQM_RIMAGETOGLTEXTURE

#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/rmaps/rmapimage.h>

#include <qgl.h>
#include <qglcolormap.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RGraphics;
using namespace RDK2::RMaps;

class RImageToGlTexture {
public:
	RImageToGlTexture();
	~RImageToGlTexture();
	
	void updateTexture(RImage* rimg);
	void updateTexture(RImage* rimg, int x, int y, int w, int h);
	void updateTexture(RMapImage* rmi);
	void updateTexture(RMapImage* rmi, int x, int y, int w, int h);
	
	inline GLuint getTextureId() { return textureId; }
	inline int getTextureWidth() { return texWidth; }
	inline int getTextureHeight() { return texHeight; }
	
	void drawMapQuad();
	
	/**
	 * @brief Returns a string representation for a GL error
	 *
	 * @param error the result of glGetError()
	 */
	static const char *glErrorToString(GLenum error);

private:
	/**
	 * @brief Prints GL errors if any happened
	 *
	 * @param when text to prepend to the error message
	 *
	 * @returns true if errors where found
	 */
	static bool printGlErrors(const char *when);
	
	void update_texture(RImage* rimg, int x, int y, int w, int h);
	bool haveToUpdateWholeMap;
	GLfloat C8ToGlR[256];
	GLfloat C8ToGlG[256];
	GLfloat C8ToGlB[256];
	GLuint textureId;
	unsigned int texWidth, texHeight;
	int lastImageWidth, lastImageHeight;
	double lastMapWidth, lastMapX, lastMapY, lastMapTheta;
	bool isMap;
	RImage::Type lastImageType;
};

}} // namespaces

#endif
