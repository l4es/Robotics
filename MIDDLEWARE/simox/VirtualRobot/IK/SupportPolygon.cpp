#include "SupportPolygon.h"

using namespace std;

namespace VirtualRobot
{


SupportPolygon::SupportPolygon(SceneObjectSetPtr contactModels)
	: contactModels(contactModels)
{
	floor =  MathTools::getFloorPlane();

	VR_ASSERT(this->contactModels);
	for (size_t i=0;i<contactModels->getSize();i++)
	{
        if (contactModels->getSceneObject(i)->getCollisionModel())
            colModels.push_back(contactModels->getSceneObject(i)->getCollisionModel());
	}

	THROW_VR_EXCEPTION_IF(colModels.size()==0,"RobotNodeSet does not contain any collision models");
}

VirtualRobot::MathTools::ConvexHull2DPtr SupportPolygon::updateSupportPolygon(float maxFloorDist)
{
	CollisionCheckerPtr colChecker = colModels[0]->getCollisionChecker();
	std::vector< CollisionModelPtr >::iterator i = colModels.begin();
	std::vector< MathTools::ContactPoint > points;
	while (i!=colModels.end())
	{
		colChecker->getContacts(floor, *i, points, maxFloorDist);
		i++;
	}

	currentContactPoints2D.clear();
	for (size_t u=0;u<points.size();u++)
	{

		Eigen::Vector2f pt2d = MathTools::projectPointToPlane2D(points[u].p,floor);
		currentContactPoints2D.push_back(pt2d);
	}
	if (currentContactPoints2D.size()<3)
		suportPolygonFloor.reset();
	else
		suportPolygonFloor = MathTools::createConvexHull2D(currentContactPoints2D);

	return suportPolygonFloor;
}

VirtualRobot::MathTools::ConvexHull2DPtr SupportPolygon::getSupportPolygon2D()
{
	return suportPolygonFloor;
}

VirtualRobot::MathTools::Plane SupportPolygon::getFloorPlane()
{
	return floor;
}


float SupportPolygon::getSquaredDistLine(Eigen::Vector2f &p, Eigen::Vector2f &pt1, Eigen::Vector2f &pt2 )
{
	//nearestPointOnLine
	Eigen::Vector2f lp = p - pt1;
	Eigen::Vector2f dir = (pt2 - pt1);
	dir.normalize();
	float lambda = dir.dot(lp);
	Eigen::Vector2f ptOnLine = pt1 + lambda*dir;

	//distPointLine
	return (ptOnLine-p).squaredNorm();
}


float SupportPolygon::getStabilityIndex(VirtualRobot::RobotNodeSetPtr rns, bool update)
{
	if (!rns)
		return 0.0f;

    if (update)
        updateSupportPolygon();

	// check if com is outside support polygon
	MathTools::ConvexHull2DPtr ch = getSupportPolygon2D();
	if (!ch || ch->vertices.size()<2)
		return 0.0f;

	Eigen::Vector2f com2D = rns->getCoM().head(2);
	if (!MathTools::isInside(com2D,ch))
	{
		//cout << "CoM outside of support polygon" << endl;
		return 0.0f;
	}

	// compute min distance center<->border
	// and min distance com to line segment
	Eigen::Vector2f center = MathTools::getConvexHullCenter(ch);
	float minDistCenter = FLT_MAX;
	float minDistCoM = FLT_MAX;
	Eigen::Vector2f pt1,pt2;
	for (size_t i=0;i<ch->vertices.size();i++)
	{
		if (i>0)
		{
			pt1 = ch->vertices[i-1];
			pt2 = ch->vertices[i];
		} else
		{
			pt1 = ch->vertices[ch->vertices.size()-1];
			pt2 = ch->vertices[0];
		}

		float d = getSquaredDistLine(center,pt1,pt2);
			//(ch->vertices[i]-center).squaredNorm();
		if (d<minDistCenter)
			minDistCenter = d;

		//distPointLine
		d = getSquaredDistLine(com2D,pt1,pt2);
		//(ptOnLine-com2D).squaredNorm();
		if (d<minDistCoM)
		{
			/*cout << "minDistCom:" << sqrtf(d) << ", com2D:" << com2D(0) << "," << com2D(1) << endl;
			cout << "minDistCom:" << sqrtf(d) << ", pt1:" << pt1(0) << "," << pt1(1) << endl;
			cout << "minDistCom:" << sqrtf(d) << ", pt2:" << pt2(0) << "," << pt2(1) << endl;
			cout << "minDistCom:" << sqrtf(d) << ", ptOnLine:" << ptOnLine(0) << "," << ptOnLine(1) << endl;*/
			minDistCoM = d;
		}
	}
	minDistCenter = sqrtf(minDistCenter);
	minDistCoM = sqrtf(minDistCoM);

	//cout << "Dist center -> border:" << minDistCenter << endl;
	//cout << "Dist com -> border:" << minDistCoM << endl;
	float res = 0.0f;
	if (fabs(minDistCenter)>1e-8)
		res = minDistCoM / minDistCenter;
	//cout << "Stability Value:" << res << endl;
	if (res>1.0f)
		res = 1.0f;
	return res;
}

VirtualRobot::SceneObjectSetPtr SupportPolygon::getContactModels()
{
	return contactModels;
}

} // namespace VirtualRobot
