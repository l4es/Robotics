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

#ifndef RDK2_RGRAPHICS_RIMAGE
#define RDK2_RGRAPHICS_RIMAGE

/*
 * Mantainers: Luca Iocchi <iocchi@dis.uniroma1.it>, Daniele Calisi <calisi@dis.uniroma1.it>
 * Past mantainers: Shahram Bahadori <bahadori@dis.uniroma1.it>
 */

#include <rdkcore/object/object.h>

namespace RDK2 { namespace RGraphics {

using namespace RDK2::Meta;

#define FOURCC(a, b, c, d) ((a) | ((b) << 8) | ((c) << 16) | ((d) << 24))

class RImage: public RDK2::Object {
public:
	/** Image types */
	enum Type {
		RGB32 = FOURCC('R', 'G', 'B', '4'),
		RGB24 = FOURCC('R', 'G', 'B', '3'),
		GREY =  FOURCC('G', 'R', 'E', 'Y'),
		C8 =    FOURCC('R', 'D', 'K', 'M'),		///< OpenRDK eight-colors map format
		JPEG =  FOURCC('J', 'P', 'E', 'G'),
		YUYV =  FOURCC('Y', 'U', 'Y', 'V')
	};

	/** Colors for OpenRDK eight-colors map format */
	enum C8Color {
		C8Unset =   0,
		C8Black =   1,
		C8Blue =    2,
		C8Green =   4,
		C8Cyan =    8,
		C8Red =     16,
		C8Magenta = 32,
		C8White =   64,
		C8Grey =    128 };
	
	/** Create an empty image: this should be used only internally by the RDK2::Object prototype database */
	RImage();
	
	/** Create an image with the given dimensions and type
	 * @param w image width
	 * @param h image height
	 * @param t image type
	 * @param buf buffer with image data, this will be COPIED; if @param buf is not given, image data is not initialized
	 */
	RImage(int w, int h, Type t, unsigned char* buf = 0);

	/** Create a new image, "clipping" an existing image
	 * @param img image to clip
	 * @param x0 left
	 * @param y0 top
	 * @param x1 right
	 * @param y1 bottom
	 */
	RImage(const RImage& img, int x0, int y0, int x1, int y1);

	/** RImage destructor, deallocates memory */
	virtual ~RImage();

	/** Copy constructor: the returned image is a deep copy of the original one, i.e., the buffer
	 * is copied, not shared
	 * @see RImage#operator=
	 */
	RImage(const RImage&);

	/** Assignment operator */
	RImage& operator=(const RImage&);

protected: 
	/** Used by constructors.  Sets
	up the buffer and rows. If buf!=0, copies the buffer from buf; else, the
	buffer is set to 'color'.*/
	virtual void initBuffer(int width, int height, Type type, void* buf, unsigned char fillValue, bool freeOldBuffer = true);

public:
	/** @name RDK::Object Interface */
	//@{
	virtual RDK2::Object* clone() const;
	//@}

	/** Changes the width, height and type of the image; the buffer is resized accordingly, the image data are lost
	 * @param w width
	 * @param h height
	 * @param t type
	 */
	virtual void setSizeAndType(size_t w, size_t h, Type t);

	/** Resizes the image buffer, the original image is placed inside the new buffer, at the given coordinates 
	 * @param w width of the new buffer
	 * @param h height of the new buffer
	 * @param x x coordinate where to place the old image
	 * @param y y coordinate where to place the old image
	 * @param fillValue fill value for the pixels not set by the old image placement
	 */
	virtual void canvasResize(size_t w, size_t h, int x = 0, int y = 0, unsigned char fillValue = 0);

	/** Fill the buffer with a specified value
	 * @param fillValue value used to fill the buffer
	 */
	virtual void fillBuffer(unsigned char fillValue);

	/** Resizes the image, using sampling technique 
	 * @param w width of the new image
	 * @param h height of the new image
	 */
	virtual void imageResize(size_t w, size_t h);
	
	/** @name Image attributes */
	//@{
	/** @return image width */
	inline size_t getWidth() const { return width; }
	
	/** @return image height */
	inline size_t getHeight() const { return height; }
	
	/** @return image type */
	inline Type getType() const { return type; }
	
	/** @return data buffer size */
	inline size_t getBufferSize() const { return bufferSize; }

	/** @return data size (i.e., the actual size of data in the buffer;
	 * for non-compressed images, it is the same of getBufferSize())
	 */
	inline size_t getDataSize() const { return dataSize; }
	
	/** Sets the data size (this make sense only for compressed types) */
	inline void setDataSize(size_t ds) { dataSize = ds; }
	
	/** @return bytes per pixel for the given type */
	static size_t getBytesPerPixel(Type t);
	
	/** @return a string representation of the type */
	static string typeToString(Type t);
	
	/** @return bytes per pixel for this image */
	inline size_t getBytesPerPixel() { return bpp; }
	
	/** @return the buffer where data resides */
	//@{
	inline unsigned char* getBuffer() { return buffer; }
	inline const unsigned char* getBuffer() const { return buffer; }
	//@}

	/** @name Pixel access */
	//@{
	inline void setPixel(size_t x, size_t y, size_t component, unsigned char value) { rows[y][x * bpp + component] = value; }
	inline unsigned char getPixel(size_t x, size_t y, size_t component) const { return rows[y][x * bpp + component]; }

	/**
	 * This function set a pixel only if the (x, y) coordinates are inside the image
	 * @return true if the (x, y) coordinates were inside the image, false otherwise
	 */
	inline bool safeSetPixel(size_t x, size_t y, size_t component, unsigned char value)
	{ return (x < width && y < height && component < bpp ? setPixel(x, y, component, value), true : false); }

	/**
	 * This function gets the pixel at coordinates (x, y) if they are inside the image, otherwise defaultValue
	 * @return the color at coordinates (x, y) if the pixel is inside the image, otherwise it returns defaultValue
	 */
	inline unsigned char safeGetPixel(size_t x, size_t y, size_t component, unsigned char defaultValue) const
	{ return (x < width && y < height && component < bpp ? getPixel(x, y, component) : defaultValue); }

	//@}

	/** Diff management (maybe will be DEPRECATED) */
	//@{
	virtual bool knowsDiffs() { return true; }
	virtual bool applyDiff(const ObjectDiff* diff);
	virtual vector<ObjectDiff*> splitInDiffs(size_t maxSize);
	//@}

	/** @name Serialization interface */
	//@{
	virtual void read(Reader* r) throw (ReadingException);
	virtual void write(Writer* w) const throw (WritingException);
	//@}
	
protected:
	/**
	 * Does the real job of serialization except for startReading() and doneReading()
	 * This method exists to be called by the subclasses.
	 * @author Matteo Leonetti (the one to blame)
	 */
	void readImpl(Reader* r) throw (ReadingException);

	/**
	 * Does the real job of serialization except for startWriting() and doneWriting()
	 * This method exists to be called by the subclasses.
	 * @author Matteo Leonetti (the one to blame)
	 */
	void writeImpl(Writer* w) const throw (WritingException);

public:
	/** Convert the image to a different type */
	virtual RImage* convertTo(Type t) const;

protected:
	/** @name Conversions implementation */
	//@{
	virtual RImage* toRGB32() const;
	virtual RImage* toRGB24() const;
	virtual RImage* toGrey() const;
	virtual RImage* toC8() const;
	virtual RImage* toJPEG() const;
	//@}

public:
	/** @name ImageMagick (Magick++) functions */
	bool loadMagick(std::string filename, double occupied = 0.5, double equal = 0.1, C8Color fillColor=C8Blue); // DEPRECATED
	bool saveMagick(std::string filename, bool trimImage = false); // DEPRECATED
	bool magickSaveToFile(const std::string& filename);
	bool magickSaveToMemory(const std::string& format, vector<unsigned char>& buffer);
	bool magickLoadFromFile(const std::string& filename);
	bool magickLoadFromMemory(const std::string& format, const vector<unsigned char>& buffer);
	
protected:
	/** @name Image attributes */
	//@{
	/** Image width */
	size_t width;

	/** Image height */
	size_t height;

	/** Image type */
	Type type;

	/** Bytes per pixel */
	size_t bpp;
	
	/** Buffer containing image data */
	unsigned char* buffer;

	/** Pointers to rows */
	unsigned char** rows;
	
	/** Buffer size */
	size_t bufferSize;
	
	/** Real data size (for non-compressed formats it is the same as buffer size) */
	size_t dataSize;
	//@}
};

uint8_t reduceC8(uint8_t color);
uint8_t expandC8(uint8_t color);

}} // namespaces

#endif
