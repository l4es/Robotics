/*
 * =====================================================================================
 *
 *       Filename:  colorobjecttable.h
 *
 *    Description:  Class implementing a LUT for color->object association
 *
 *        Version:  1.0
 *        Created:  03/22/2009 12:57:05 PM
 *
 *         Author:  Luca Marchetti (lm), ximilian@gmail.com
 *
 * =====================================================================================
 */


#ifndef  COLOROBJECTTABLE_INC
#define  COLOROBJECTTABLE_INC


#include	<map>
#include	<string>
#include	<rdkcore/geometry/point.h>
#include	<rdkcore/rgraphics/color.h>
#include <rdkcore/object/object.h>

using RDK2::Geometry::Point3od;
using RDK2::RGraphics::RgbColor;

struct ObjectInfo
{
	std::string id;
	double width, height, depth;
};

/*
 * =====================================================================================
 *        Class:  ColorObjectTable
 *  Description:  Class implementing a LUT for color<->object association
 * =====================================================================================
 */

typedef std::map<RgbColor,ObjectInfo> ColorObjectTable;



/*
 * =====================================================================================
 *        Class:  RObjectInfo
 *  Description:  R-wrapper for an ObjectInfo
 * =====================================================================================
 */
struct RObjectInfo: ObjectInfo, RDK2::Object
{
	RDK2_DEFAULT_CLONE(RObjectInfo);
	RObjectInfo(): ObjectInfo() {}
	RObjectInfo(const ObjectInfo& o);
	virtual void read(RDK2::Reader* r) throw (RDK2::ReadingException);
	virtual void write(RDK2::Writer* w) const throw (RDK2::WritingException);

	virtual void readImpl(RDK2::Reader* r) throw (RDK2::ReadingException);
	virtual void writeImpl(RDK2::Writer* w) const throw (RDK2::WritingException);
}; /* -----  end of class RObjectInfo  ----- */

/*
 * =====================================================================================
 *        Class:  RColorObjectTable
 *  Description:  R-wrapper for a ColorObjectTable
 * =====================================================================================
 */
class RColorObjectTable: public ColorObjectTable, public RDK2::Object
{
	public:
		RColorObjectTable(): ColorObjectTable() {}
		RColorObjectTable(const ColorObjectTable& c);
		RDK2_DEFAULT_CLONE(RColorObjectTable);
		virtual void read(RDK2::Reader* r) throw (RDK2::ReadingException);
		virtual void write(RDK2::Writer* w) const throw (RDK2::WritingException);

}; /* -----  end of class RColorObjectTable  ----- */


#endif   /* ----- #ifndef COLOROBJECTTABLE_INC  ----- */

