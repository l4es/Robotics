#include "rnaosensors.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RNaoSensors"

#include <sstream>

using namespace std;

namespace RDK2 {

RDK2_FACTORY(RNaoSensors)

Object* RNaoSensors::clone() const {
	RNaoSensors* rns = new RNaoSensors(*this);
	return rns;
}

void RNaoSensors::read(RDK2::Serialization::Reader*r) throw (ReadingException)
{
	r->startReading(getClassName());
		sensors.timestamp = r->read_f64();

		sensors.battery.charge = r->read_f32();
		sensors.battery.current = r->read_f32();
		sensors.battery.temperature = r->read_f32();

		for (size_t i=0; i<JOINTS_VALUES_SIZE; ++i)
		{
			sensors.jointsValues[i] = r->read_f32();
		}

		sensors.gyr.x = r->read_f32();
		sensors.gyr.y = r->read_f32();
		sensors.gyr.ref = r->read_f32();

		sensors.acc.x = r->read_f32();
		sensors.acc.y = r->read_f32();
		sensors.acc.z = r->read_f32();
		
		sensors.angle.x = r->read_f32();
		sensors.angle.y = r->read_f32();

		sensors.us.left = r->read_f32();
		sensors.us.right = r->read_f32();

		sensors.fsr.left.x = r->read_f32();
		sensors.fsr.left.y = r->read_f32();
		sensors.fsr.left.weight = r->read_f32();
		sensors.fsr.right.x = r->read_f32();
		sensors.fsr.right.y = r->read_f32();
		sensors.fsr.right.weight = r->read_f32();

		sensors.bumpers.left.left = r->read_f32();
		sensors.bumpers.left.right = r->read_f32();
		sensors.bumpers.right.left = r->read_f32();
		sensors.bumpers.right.right = r->read_f32();

		r->doneReading();
}

void RNaoSensors::write(RDK2::Serialization::Writer*w) const throw (WritingException)
{
	w->startWriting(getClassName());
		w->write_f64(sensors.timestamp);

		w->write_f32(sensors.battery.charge);
		w->write_f32(sensors.battery.current);
		w->write_f32(sensors.battery.temperature);

		for (size_t i=0; i<JOINTS_VALUES_SIZE; ++i)
		{
			w->write_f32(sensors.jointsValues[i]);
		}

		w->write_f32(sensors.gyr.x);
		w->write_f32(sensors.gyr.y);
		w->write_f32(sensors.gyr.ref);

		w->write_f32(sensors.acc.x);
		w->write_f32(sensors.acc.y);
		w->write_f32(sensors.acc.z);
		
		w->write_f32(sensors.angle.x);
		w->write_f32(sensors.angle.y);

		w->write_f32(sensors.us.left);
		w->write_f32(sensors.us.right);

		w->write_f32(sensors.fsr.left.x);
		w->write_f32(sensors.fsr.left.y);
		w->write_f32(sensors.fsr.left.weight);
		w->write_f32(sensors.fsr.right.x);
		w->write_f32(sensors.fsr.right.y);
		w->write_f32(sensors.fsr.right.weight);

		w->write_f32(sensors.bumpers.left.left);
		w->write_f32(sensors.bumpers.left.right);
		w->write_f32(sensors.bumpers.right.left);
		w->write_f32(sensors.bumpers.right.right);

	w->doneWriting();
}

string RNaoSensors::getStringRepresentation() const {
	ostringstream stream;
	stream << "[" << sensors.timestamp << "] ";
	stream << "batt:<" << sensors.battery.charge<< " " << sensors.battery.current<< " "<<  sensors.battery.temperature<<">";
	stream << "joints:<";
	for (size_t k=0; k<JOINTS_VALUES_SIZE-1; k++) { stream << sensors.jointsValues[k] << ","; }
	stream << sensors.jointsValues[JOINTS_VALUES_SIZE-1] << "> ";
	stream << "gyr:<"<<sensors.gyr.x<<","<<sensors.gyr.y<<","<<sensors.gyr.ref<<"> ";
	stream << "acc:<"<<sensors.acc.x<<","<<sensors.acc.y<<","<<sensors.acc.z<<"> ";
	stream << "ang:<"<<sensors.angle.x<<","<<sensors.angle.y<<"> ";
	stream << "fsr:<";
	stream << sensors.fsr.left.x << "," << sensors.fsr.left.y << "," << sensors.fsr.left.weight;
	stream << sensors.fsr.right.x << "," << sensors.fsr.right.y << "," << sensors.fsr.right.weight << "> ";
	stream << "us:<"<<sensors.us.left<<","<<sensors.us.right<<"> ";
	stream << "bump:<"<<sensors.bumpers.left.left<<","<<sensors.bumpers.left.right<<"," <<sensors.bumpers.right.left<<","<<sensors.bumpers.right.right<<"> ";
	return stream.str();
}

ostream &operator<<(ostream &stream, RNaoSensors& rs) {

	stream << rs.getStringRepresentation();
	return stream;

}
	
} // namespace
