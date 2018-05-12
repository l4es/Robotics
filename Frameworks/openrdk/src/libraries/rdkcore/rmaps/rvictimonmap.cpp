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

#include "rvictimonmap.h"

namespace RDK2 { namespace RMaps {

RDK2_FACTORY(RVictimOnMap);

using namespace std;

RVictimOnMap::RVictimOnMap() :
 	zPosition(0.), confirmedOp(false), confirmedAI(false),
	falsePositiveOp(false), falsePositiveAI(false), reliabilityOp(0.), reliabilityAI(0.),
	temperatureOp(0.), temperatureAI(0.)
{
	snapshotOp = new RImage(3, 3, RImage::RGB32);
	snapshotAI = new RImage(3, 3, RImage::RGB32);
}

void RVictimOnMap::write(Writer* w) const throw (WritingException)
{
	w->startWriting(getClassName());
		w->write_i8(confirmedOp);		w->write_i8(confirmedAI);	
		w->write_i8(falsePositiveOp);	w->write_i8(falsePositiveAI);
		w->write_f32(reliabilityOp);	w->write_f32(reliabilityAI);
		w->writeString(idOp);			w->writeString(idAI);
		w->writeString(descriptionOp);	w->writeString(descriptionAI);
		w->writeString(motionOp);		w->writeString(motionAI);
		w->write_f32(temperatureOp);	w->write_f32(temperatureAI);
		w->writeString(soundOp);		w->writeString(soundAI);
		w->writeString(situationOp);	w->writeString(situationAI);
		w->writeString(statusOp);		w->writeString(statusAI);
		w->writeString(partsOp);		w->writeString(partsAI);
		w->writeObject(true, snapshotOp); w->writeObject(true, snapshotAI);
		w->write_f32(pose.x);
		w->write_f32(pose.y);
		w->write_f32(pose.theta);
		w->write_f32(theta);
		w->write_f32(dist);
		w->write_f32(zPosition);
		w->write_f32(robotPose.x);
		w->write_f32(robotPose.y);
		w->write_f32(robotPose.theta);
	w->doneWriting();
}

string RVictimOnMap::getLogLine()
{
	//double victimDist = robotPose.distTo(pose);
	//double victimTheta = atan2(pose.y - robotPose.y, pose.x - robotPose.x) - robotPose.theta;
	ostringstream oss;
	//oss << "VICTIM " << victimTheta << " " << victimDist << " "
	oss << "VICTIM " << theta << " " << dist << " "
		<< robotPose.x << " " << robotPose.y << " " << robotPose.theta << " " << idAI << " "
		<< timestamp.getMsFromMidnight() << " unknown " << timestamp.getMsFromMidnight();
	return oss.str();
}

void RVictimOnMap::read(Reader* r) throw (ReadingException)
{
	r->startReading(getClassName());
		confirmedOp = r->read_i8();			confirmedAI = r->read_i8();
		falsePositiveOp = r->read_i8();		falsePositiveAI = r->read_i8();
		reliabilityOp = r->read_f32();		reliabilityAI = r->read_f32();
		idOp = r->readString();				idAI = r->readString();
		descriptionOp = r->readString();	descriptionAI = r->readString();
		motionOp = r->readString();			motionAI = r->readString();
		temperatureOp = r->read_f32();		temperatureAI = r->read_f32();
		soundOp = r->readString();			soundAI = r->readString();
		situationOp = r->readString();		situationAI = r->readString();
		statusOp = r->readString();			statusAI = r->readString();
		partsOp = r->readString();			partsAI = r->readString();
		snapshotOp = dynamic_cast<RImage*>(r->readObject());
		snapshotAI = dynamic_cast<RImage*>(r->readObject());
		pose.x = r->read_f32();
		pose.y = r->read_f32();
		pose.theta = r->read_f32();
		theta = r->read_f32();
		dist = r->read_f32();
		zPosition = r->read_f32();
		robotPose.x = r->read_f32();
		robotPose.y = r->read_f32();
		robotPose.theta = r->read_f32();
	r->doneReading();
}

RDK2::Object* RVictimOnMap::clone() const
{
	RVictimOnMap* r = (RVictimOnMap*) (new RVictimOnMap(*this));
	r->snapshotOp = (RImage*) (this->snapshotOp->clone());
	r->snapshotAI = (RImage*) (this->snapshotAI->clone());
	return r;
}

RVictimOnMap::~RVictimOnMap()
{
	delete snapshotOp;
	delete snapshotAI;
}

double RVictimOnMap::computeTemperatureMean()
{
	double temperatureMean = 0.;
	for (size_t i = 0; i < temperatureCache.size(); i++) {
		temperatureMean += temperatureCache[i];
	}
	temperatureMean /= temperatureCache.size();
	return temperatureMean;
}

double RVictimOnMap::computeSeenPartsMean()
{
	double seenPartsMean = 0.;
	for (size_t i = 0; i < seenPartsCache.size(); i++) {
		seenPartsMean += seenPartsCache[i];
	}
	seenPartsMean /= seenPartsCache.size();
	return seenPartsMean;
}

Point2od RVictimOnMap::computePoseMean()
{
	Point2od poseMean(0., 0., 0.);
	if (poseCache.size() > 0) {
		for (size_t i = 0; i < poseCache.size(); i++) {
			poseMean.x += poseCache[i].x;
			poseMean.y += poseCache[i].y;
		}
		poseMean.x /= poseCache.size();
		poseMean.y /= poseCache.size();
	}
	return poseMean;
}

}} // namespace
