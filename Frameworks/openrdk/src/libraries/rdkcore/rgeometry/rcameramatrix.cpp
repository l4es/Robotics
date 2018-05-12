#include "rcameramatrix.h"

RDK2_FACTORY(RCameraMatrix);

RCameraMatrix::RCameraMatrix() : CameraMatrix() {}

RCameraMatrix::RCameraMatrix(const CameraMatrix &cm) : CameraMatrix(cm) {}

RDK2::Object* RCameraMatrix::clone() const
{
	return new RCameraMatrix(*this);
}

void RCameraMatrix::read(RDK2::Serialization::Reader*r)
	throw (RDK2::ReadingException)
{
	r->startReading(getClassName());
	T.x = r->read_f32();
	T.y = r->read_f32();
	T.z = r->read_f32();
	for (int i=0; i<3; ++i)
	{
		for (int j=0; j<3; ++j)
		{
			R[i][j]= r->read_f32();
		}
	}
	r->doneReading();
}

void RCameraMatrix::write(RDK2::Serialization::Writer*w) const
	throw (RDK2::WritingException)
{
	w->startWriting(getClassName());
	const RDK2::Geometry::Point3d& T = getT();
	const RDK2::Geometry::DMatrixD& R = getR();
	w->write_f32(T.x);
	w->write_f32(T.y);
	w->write_f32(T.z);
	for (int i=0; i<3; ++i)
	{
		for (int j=0; j<3; ++j)
		{
			w->write_f32(R[i][j]);
		}
	}

	w->doneWriting();
}
