#ifndef __rcameramatrix_h__
#define __rcameramatrix_h__

#include <rdkcore/geometry/cameramatrix.h>
#include <rdkcore/object/object.h>

class RCameraMatrix :  public RDK2::Object, public CameraMatrix {
public:
	RCameraMatrix();
	RCameraMatrix(const CameraMatrix &cm);
	RDK2::Object* clone() const;
		virtual void read(RDK2::Serialization::Reader*r)
				throw (RDK2::ReadingException);
			virtual void write(RDK2::Serialization::Writer*w) const
				throw (RDK2::WritingException);

};

#endif
