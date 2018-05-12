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
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/filesystem/filesystem.h>
#define LOGGING_MODULE "PGMFile"
#include "pgmfile.h"

namespace RDK2 { namespace Geometry {
	using namespace std;
	using namespace RDK2::Filesystem;
	
	void PGMFile::load(const std::string&file) throw(runtime_error) {
		ifstream ifs;
		fsOpenOrException(ifs, file);
		load(ifs);
	}
	
	void PGMFile::load(std::istream&is) throw(runtime_error) {
		string magicNumber;
		is >> magicNumber;
	
		if(magicNumber== "P5") {
			this->type = P5;
		} else {
			string error = string("Can not read file with magic number ")+quote(magicNumber)+".";
			RDK_ERROR_STREAM(error);
			throw runtime_error(error);
		}
		
		this->comment = RDK2::TextUtils::readCommentLines(is);
		RDK_DEBUG_STREAM("Read comment: "<<quote(comment));
		
		if(!(is>>width)) {
			string error = "Could not read width.";
			RDK_ERROR_STREAM(error);
			throw runtime_error(error);
		}	
		RDK_TRACE_PRINTF("Read width=%d", width);
	
		RDK2::TextUtils::consumeWhitespace(is);	
		
		if(!(is>>height)) {
			string error = "Could not read height.";
			RDK_ERROR_STREAM(error);
			throw runtime_error(error);
		}	
		RDK_TRACE_PRINTF("Read height=%d", height);
	
		if(width<=0 || height <=0) {
			string error = string("Bad width, height =")+toString(width)+","+toString(height);
			RDK_ERROR_STREAM(error);
			throw runtime_error(error);
		}
	
		RDK2::TextUtils::consumeWhitespace(is);	
		if(!(is>>this->maxVal)) {
			string error = "Could not read maxVal.";
			RDK_ERROR_STREAM(error);
			throw runtime_error(error);
		}	
		RDK_TRACE_PRINTF("Read maxVal=%d", maxVal);
	
		if(type == P5) {
			int EXPECT_MAXVAL = 255;
			if(maxVal!=EXPECT_MAXVAL) {
				string error = "I expected a maxVal of " + toString(EXPECT_MAXVAL)
					+ " instead of " + toString(maxVal);
				RDK_ERROR_STREAM(error);
				throw runtime_error(error);
			}
			is.get();
			
			int rows    = height;
			int columns = width;
			
			this->buffer = DMatrix<int>(rows,columns);
			//int cold;
			for(int y=0;y<height;y++)
				for(int x=0;x<width;x++)  {
					int c = is.get();
					buffer[y][x] = c;
					
					/*
					if(c!=cold) {
						RDK_INFO_STREAM("Read " << buffer[y][x]);
					}
					cold = c;*/
				}
		} else assert(false);
	}
	
	bool PGMFile::load_(const std::string&file) {
		try { load(file); return true; } catch(exception&e) {
			RDK_ERROR_STREAM(e.what());
			return false;
		}
	}
	
	
	
	Viewport PGMFile::getViewport(double mapResolution) 
		throw(runtime_error, invalid_argument)
	{
		if(mapResolution <0) {
			string error = string("Invalid map resolution: ")+toString(mapResolution);
			throw invalid_argument(error);
		}
		
		if(mapResolution == 0 &&
			!readResolution(mapResolution)) {
			string error = string("Could not read resolution inside comment: ")+comment;
			throw runtime_error(error);
		} else {
			RDK_DEBUG_STREAM("Map resolution: "<<mapResolution);
		}
		
		Point2d rmin(0,0);
		Point2d rmax(mapResolution * width, mapResolution * height); 
		return Viewport(rmin,rmax);
	}
	
	
	template<class T> bool readVariable(istream&is, cstr name, T&t) {
		bool found = false;
		string token;
		while(is>>token) 
			if(token==name) {
				is>>t;
				if(!is.fail()) {
					found = true; 
					break; 
				}
			}
		return found;
	}
	
	bool PGMFile::readResolution(double&mapResolution){
		istringstream iss(comment);
		if(!readVariable<double>(iss, "resolution", mapResolution)) {
			return false;
		}
		if(mapResolution<=0) {
			string error = string("Invalid mapResolution: ") + toString(mapResolution);
			RDK_ERROR_STREAM(error);
			return false;
		}
		return true;
	}
	
		
		
}} // ns


/*Each PGM image consists of the following:

   1. A "magic number" for identifying the file type. A pgm image's magic number is the two characters "P5".
   2. Whitespace (blanks, TABs, CRs, LFs).
   3. A width, formatted as ASCII characters in decimal.
   4. Whitespace.
   5. A height, again in ASCII decimal.
   6. Whitespace.
   7. The maximum gray value (Maxval), again in ASCII decimal. Must be less than 65536, and more than zero.
   8. Newline or other single whitespace character.
   9. A raster of Height rows, in order from top to bottom. Each row consists of Width gray values, in order from left to right. Each gray value is a number from 0 through Maxval, with 0 being black and Maxval being white. Each gray value is represented in pure binary by either 1 or 2 bytes. If the Maxval is less than 256, it is 1 byte. Otherwise, it is 2 bytes. The most significant byte is first.

	  A row of an image is horizontal. A column is vertical. The pixels in the image are square and contiguous.
  10. Each gray value is a number proportional to the intensity of the pixel, adjusted by the CIE Rec. 709 gamma transfer function. (That transfer function specifies a gamma number of 2.2 and has a linear section for small intensities). A value of zero is therefore black. A value of Maxval represents CIE D65 white and the most intense value in the image and any other image to which the image might be compared.
  11. Note that a common variation on the PGM format is to have the gray value be "linear," i.e. as specified above except without the gamma adjustment. pnmgamma takes such a PGM variant as input and produces a true PGM as output.
  12. In the transparency mask variation on PGM, the value represents opaqueness. It is proportional to the fraction of intensity of a pixel that would show in place of an underlying pixel. So what normally means white represents total opaqueness and what normally means black represents total transparency. In between, you would compute the intensity of a composite pixel of an "under" and "over" pixel as under * (1-(alpha/alpha_maxval)) + over * (alpha/alpha_maxval). Note that there is no gamma transfer function in the transparency mask.
  13. Characters from a "#" to the next end-of-line, before the maxval line, are comments and are ignored. 
  
 */
