#ifndef __cameramatrix_h__
#define __cameramatrix_h__

#include <rdkcore/geometry/dmatrix.h>
#include <rdkcore/geometry/dmatrix_point.h>
#include <rdkcore/geometry/point.h>

class CameraMatrix
{
	public:
		/** Roll, pitch and yaw are defined as http://en.wikipedia.org/wiki/Flight_dynamics */
		CameraMatrix(const float& x=0., const float& y=0., const float& z=0.,
				const float& roll=0., const float& pitch=0., const float& yaw=0.
				);

		CameraMatrix(const CameraMatrix& cm);
		CameraMatrix(const float* m);

		CameraMatrix& operator=(const CameraMatrix& cameramatrix);
		bool operator==(const CameraMatrix& rhs);
		~CameraMatrix();

		void rotateRoll(const float& r);
		void rotatePitch(const float& p);
		void rotateYaw(const float& y);

		inline const RDK2::Geometry::Point3d& getT() const { return T; };
		inline const RDK2::Geometry::DMatrixD& getR() const { return R; };
		inline RDK2::Geometry::Point3d& getT() { return T; };
		inline RDK2::Geometry::DMatrixD& getR() { return R; };

		static CameraMatrix getTestCase();
		std::string toString() const;

	protected:
		RDK2::Geometry::Point3d T;
		RDK2::Geometry::DMatrixD R;
};


#endif
