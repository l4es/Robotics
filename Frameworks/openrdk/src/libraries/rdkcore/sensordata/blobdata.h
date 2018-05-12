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

#ifndef H_BLOBDATA
#define H_BLOBDATA

#include "sensordata.h"

namespace RDK2 { namespace SensorData {

	struct BlobData : public BaseSensorData {
	
		struct Blob {
			unsigned int id;     ///< Blob id
			unsigned int color;  ///< Blob color. The color is stored as packed 32-bit RGB, i.e., 0x00RRGGBB. 
			unsigned int area;   ///< Blob dimension (square pixels)
			unsigned int x;      ///< Blob center's x
			unsigned int y;      ///< Blob center's y
			unsigned int left;   ///< Blob bounding box
			unsigned int right;  ///< Blob bounding box
			unsigned int top;    ///< Blob bounding box
			unsigned int bottom; ///< Blob bounding box
			double       range;  ///< Range to blob center, in m
		};

		double         panAngle;    ///< Pan angle for the scan
		double         tiltAngle;   ///< Tilt angle for the scan
		double         zoomAngle;   //< FOV angle
		unsigned short width;       ///< Width of the image processed
		unsigned short height;      ///< Height of the image processed

	
		BlobData(std::string logtag="BLOB"):
			BaseSensorData(logtag),
			panAngle(0.),
			tiltAngle(0.),
			zoomAngle(0.),
			width(0.),
			height(0.)
		{}
	
		typedef std::vector< Blob > BlobSet;
		BlobSet blobSet;
	};

}} // namespace

#endif
