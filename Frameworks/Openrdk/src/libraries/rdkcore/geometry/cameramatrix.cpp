#include "cameramatrix.h"
#include <rdkcore/geometry/utils.h>

using RDK2::Geometry::Point3d;
using RDK2::Geometry::DMatrixD;

CameraMatrix::CameraMatrix
( const float& x, const float& y, const float& z,
	const float& roll, const float& pitch, const float& yaw
	):
	T(x,y,z),
	R(DMatrixD::I(3))
{
	if (roll)
	{
		rotateRoll(roll);
		rotatePitch(pitch);
		rotateYaw(yaw);
	}
}

CameraMatrix::CameraMatrix(const float* m): R(DMatrixD::I(3))
{
	/* :TODO:30/12/09 14:11:19:lm: insert check on m size */
	T.x=m[3];
	T.y=m[7];
	T.z=m[11];
	R[0][0]=m[0];
	R[0][1]=m[1];
	R[0][2]=m[2];
	R[1][0]=m[4];
	R[1][1]=m[5];
	R[1][2]=m[6];
	R[2][0]=m[8];
	R[2][1]=m[9];
	R[2][2]=m[10];
}


CameraMatrix::CameraMatrix(const CameraMatrix& cm): T(cm.getT()), R(cm.getR())
{
}

CameraMatrix& CameraMatrix::operator=(const CameraMatrix& cameraMatrix)
{
	if (this == &cameraMatrix)
	{
		return *this;
	}
	T = cameraMatrix.getT();
	R = cameraMatrix.getR();

	return *this;
}

bool CameraMatrix::operator==(const CameraMatrix& rhs)
{
	return this->T == rhs.getT() && this->R == rhs.getR();
}

void CameraMatrix::rotateRoll(const float& r)
{
	float s = sin(r),
				c = cos(r);

	DMatrixD roll(DMatrixD::I(3));
	roll[1][1] = c;
	roll[1][2] = s;
	roll[2][1] = -s;
	roll[2][2] = c;

	R = R * roll;
}


void CameraMatrix::rotatePitch(const float& p)
{
	float s = sin(p),
				c = cos(p);

	DMatrixD pitch(DMatrixD::I(3));
	pitch[0][0] = c;
	pitch[2][0] = s;
	pitch[0][2] = -s;
	pitch[2][2] = c;

	R = R * pitch;
}

void CameraMatrix::rotateYaw(const float& y)
{
	float s = sin(y),
				c = cos(y);

	DMatrixD yaw(DMatrixD::I(3));
	yaw[0][0] = c;
	yaw[1][0] = -s;
	yaw[0][1] = s;
	yaw[1][1] = c;

	R = R * yaw;
}

CameraMatrix::~CameraMatrix()
{}

CameraMatrix CameraMatrix::getTestCase()
{
	CameraMatrix cm;
	cm.rotatePitch(RDK2::Geometry::deg2rad(10));
	//cm.rotateYaw(RDK2::Geometry::deg2rad(10));
	//cm.rotateRoll(RDK2::Geometry::deg2rad(10));
	return cm;
}

std::string CameraMatrix::toString() const
{
	std::ostringstream oss;
	oss << "(" << " T: "
		<< this->T.x << "," << this->T.y << "," << this->T.z <<" ; R: " << this->R[0][0] << "," << this->R[0][1] << "," << this->R[0][2] << "," << this->R[1][0] << "," << this->R[1][1] << "," << this->R[1][2] << "," << this->R[2][0] << "," << this->R[2][1] << "," << this->R[2][2] << " ) " ;
	return oss.str();
}

