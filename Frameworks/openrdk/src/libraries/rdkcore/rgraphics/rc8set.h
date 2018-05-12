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

#ifndef RDK2_RGRAPHICS_RC8SET
#define RDK2_RGRAPHICS_RC8SET

#include <rdkcore/object/object.h>
#include <rdkcore/rgraphics/rimage.h>

namespace RDK2 { namespace RGraphics {

	class RC8Set : public RDK2::Object {
		public:
			RC8Set() : mask(0) { }
			RC8Set(uint8_t initialMask) : mask(initialMask) { }
			virtual ~RC8Set() { }
		
			RDK2_DEFAULT_CLONE(RC8Set);
			virtual void read(Reader* r) throw (ReadingException);
			virtual void write(Writer* w) const throw (WritingException);
		
			inline bool contains(uint8_t color) const { return (color & mask); }
			inline void set(RImage::C8Color color) { mask |= color; }
			inline void unset(RImage::C8Color color) { mask &= ~color; }
			inline void clear() { mask = 0; }
		
		private:
			uint8_t mask;
	};

}} // namespace

#endif
