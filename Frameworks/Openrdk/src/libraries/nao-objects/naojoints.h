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

#ifndef RDK_NAOJOINTS
#define RDK_NAOJOINTS

#include <rdkcore/object/object.h>

#include <vector>

#define INTERPOLATION_LINEAR 0
#define INTERPOLATION_SMOOTH 1

namespace Nao {

class NaoJoints {
public:
	enum EJoints {
		/* HEAD */ HEAD_YAW = 0, HEAD_PITCH, 
		/* LEFT ARM */ L_SHOULDER_PITCH, L_SHOULDER_ROLL, L_ELBOW_YAW, L_ELBOW_ROLL,
		/* LEFT LEG */ L_HIP_YAW_PITCH, L_HIP_ROLL, L_HIP_PITCH, L_KNEE_PITCH, L_ANKLE_PITCH, L_ANKLE_ROLL,
		/* RIGHT LEG */ R_HIP_YAW_PITCH, R_HIP_ROLL, R_HIP_PITCH, R_KNEE_PITCH, R_ANKLE_PITCH, R_ANKLE_ROLL,
		/* RIGHT ARM */ R_SHOULDER_PITCH, R_SHOULDER_ROLL, R_ELBOW_YAW, R_ELBOW_ROLL,
		NAO_JOINTS_COUNT
	};

	enum EChains {
		None, Body, Head, LArm, LLeg, RLeg, RArm,    // LI do not change!!!
		NAO_CHAINS_COUNT };

	enum EJointsStatus { STATUS_OK, STATUS_SEND, STATUS_ERROR };

	NaoJoints();
	virtual ~NaoJoints() { }
	
	//static float initialPose[];
	
	static NaoJoints getZeroPose();
	static NaoJoints getSafePose();
	//added by gianluca
	std::vector<float> getInitPose();
	
	float &operator[](size_t index);
	float operator[](size_t index) const;
	
	std::vector<float> &getValues();
	const std::vector<float> &getValues() const;
	std::vector<float> getHeadChain();
	std::vector<float> getLeftArmChain();
	std::vector<float> getLeftLegChain();
	std::vector<float> getRightLegChain();
	std::vector<float> getRightArmChain();
	
	bool operator==(NaoJoints theOtherOperator);
	NaoJoints operator+(NaoJoints secondElement);
	NaoJoints operator-(NaoJoints secondElement);
	int status;

private:
	friend std::istream &operator>>(std::istream &stream, NaoJoints& joints);
	friend std::ostream &operator<<(std::ostream &stream, NaoJoints& joints);
	friend NaoJoints operator*(NaoJoints point, float scalar);
	friend NaoJoints operator*(float scalar, NaoJoints point);

protected:
	std::vector<float> value;
}; // NaoJoints class

} //namespace

#endif
