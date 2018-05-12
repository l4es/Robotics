#include "naojoints.h"

#include <rdkcore/geometry/utils.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "NaoJoints"

#include <iostream>
#include <iterator>
#include <algorithm>
#include <functional>
#include <iomanip>

using namespace std;
using RDK2::Geometry::deg2rad;


const float kneeAngle = deg2rad(60.);
const float wideAngle = deg2rad(0.);
const float torsoAngle = deg2rad(0.);

namespace Nao {

	NaoJoints::NaoJoints() : status(STATUS_OK), value(getInitPose())
	{
		/* MOVED IN GETINITPOSE()
			 float initialPose[] =
			 {
			 0, 0, 1.71344, 0.317496, -1.38371, -1.53396, 0, 0, -0.444818, 0.895814, -0.451038, -0.00762804, 0, 0, -0.451038, 0.897432, -0.455556, 0, 1.71966, -0.316046, 1.38363, 1.53396
			 0.0, 0.0,
			 deg2rad(45), deg2rad(15), deg2rad(-80), deg2rad(-80),
			 0.0, wideAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2, -wideAngle,
			 0.0, -wideAngle, -kneeAngle/2-torsoAngle, kneeAngle, -kneeAngle/2, wideAngle,
			 deg2rad(45), deg2rad(-15), deg2rad(80), deg2rad(80) };

			 copy(initialPose, initialPose + (sizeof(initialPose)/sizeof(float)), back_inserter(value));*/
	}

	bool NaoJoints::operator==(NaoJoints theOtherOperator){
		if(value.size()!=theOtherOperator.getValues().size()) return false;//FIXME ERRORE

		vector<float>::iterator itOp2=theOtherOperator.getValues().begin();

		for(vector<float>::iterator itOp1=value.begin();itOp1!=value.end();itOp1++){
			if((*itOp1)==(*itOp2)) itOp2++;
			else
				return false;
		}
		return true;
	}

	NaoJoints NaoJoints::operator+(NaoJoints secondElement)
	{
		NaoJoints result;
		if(secondElement.value.size()==value.size()){

			vector<float>::iterator itOp2=secondElement.value.begin();
			vector<float>::iterator itRes=result.value.begin();
			for(vector<float>::iterator itOp1=value.begin();itOp1!=value.end();itOp1++){
				*itRes=*itOp1+*itOp2;
				itOp2++; itRes++;
			}
		}
		return result;
	}


	NaoJoints NaoJoints::operator-(NaoJoints secondElement)
	{
		NaoJoints result;

		if(secondElement.value.size()==value.size()){

			vector<float>::iterator itOp2=secondElement.value.begin();
			vector<float>::iterator itRes=result.value.begin();

			for(vector<float>::iterator itOp1=value.begin();itOp1!=value.end();itOp1++){
				*itRes=*itOp1-*itOp2;
				itOp2++; itRes++;
			}
		}	

		return result;

	}
	NaoJoints operator*(NaoJoints point,float scalar){

		NaoJoints result;
		vector<float>pointValues=point.value;
		for(vector<float>::iterator pointIt=pointValues.begin();pointIt!=pointValues.end();pointIt++)
			(*pointIt)*=scalar;
		result.value=pointValues;
		return result;	
	}
	NaoJoints operator*(float scalar,NaoJoints point){

		NaoJoints result;
		vector<float>pointValues=point.value;
		for(vector<float>::iterator pointIt=pointValues.begin();pointIt!=pointValues.end();pointIt++)
			(*pointIt)*=scalar;
		result.value=pointValues;
		return result;		
	}


	//added by gianluca
	vector<float> NaoJoints::getInitPose() {

		//  v.reserve(22);
		static float init[] = {0, 0, 1.71344, 0.317496, -1.38371, -1.53396, 0, 0, -0.444818, 0.895814, -0.451038, -0.00762804, 0, 0, -0.451038, 0.897432, -0.455556, 0, 1.71966, -0.316046, 1.38363, 1.53396};
		vector<float> v(init,init + sizeof(init) / sizeof(float));
		//  for(int i = 0; i < NAO_JOINTS_COUNT; i++) v[i] = init[i];
		return v;
	}

	float &NaoJoints::operator[](size_t index) {
		return value[index];
	}

	float NaoJoints::operator[](size_t index) const{
		return value[index];
	}

	vector<float> &NaoJoints::getValues() {
		return value;
	}

	const vector<float> &NaoJoints::getValues() const
	{
		return value;
	}

	vector<float> NaoJoints::getHeadChain() {
		vector<float>::iterator first = value.begin();
		vector<float>::iterator end   = first + 2;
		vector<float> chain(first,end);
		return chain;
	}

	vector<float> NaoJoints::getLeftArmChain() {
		vector<float>::iterator first = value.begin() + 2;
		vector<float>::iterator end   = first + 4;
		vector<float> chain(first,end);
		return chain;
	}

	vector<float> NaoJoints::getLeftLegChain() {
		vector<float>::iterator first = value.begin() + 6;
		vector<float>::iterator end   = first + 6;
		vector<float> chain(first,end);
		return chain;
	}

	vector<float> NaoJoints::getRightLegChain() {
		vector<float>::iterator first = value.begin() + 12;
		vector<float>::iterator end   = first + 6;
		vector<float> chain(first,end);
		return chain;
	}

	vector<float> NaoJoints::getRightArmChain() {
		vector<float>::iterator first = value.begin() + 18;
		vector<float>::iterator end   = first + 4;
		vector<float> chain(first,end);
		return chain;
	}

	NaoJoints NaoJoints::getSafePose()
	{
		float safePose[] = { 0.30, 0.26, 1.09, 0.11, -0.17, -1.17, -0.80, 0.01, -1.18, 2.12,
			-0.53, 0.15, -0.80, -0.04, -0.36, 2.14, -1.22, 0.03, 0.30, 0.26, 1.09, 0.11 };
		NaoJoints j;
		j.value.clear();
		copy(safePose, safePose + (sizeof(safePose)/sizeof(float)), back_inserter(j.value));
		return j;
	}

	NaoJoints NaoJoints::getZeroPose()
	{
		NaoJoints j;
		j.value = vector<float>(NAO_JOINTS_COUNT, 0.0);
		return j;
	}

	struct Extractor : public unary_function<void,float> {

		Extractor(istream& stream) : stream(stream) {}

		float operator() (void) {
			float f;
			stream >> f;
			return f;
		}

		istream& stream;
	};

	istream &operator>>(istream &stream, NaoJoints& joints) {

		generate(joints.value.begin(),joints.value.end(),Extractor(stream));

		return stream;
	}


	struct Inserter : public unary_function<float,void> {

		Inserter(ostream& stream) : stream(stream) {}

		void operator() (float value) {
			stream << setiosflags(ios::fixed) << setprecision(2) << value << " ";
		}

		ostream &stream;
	};


	ostream &operator<<(ostream &stream, NaoJoints& joints) {

		for_each(joints.value.begin(), joints.value.end(), Inserter(stream));
		stream << std::endl;

		return stream;

	}

}
// namespace
