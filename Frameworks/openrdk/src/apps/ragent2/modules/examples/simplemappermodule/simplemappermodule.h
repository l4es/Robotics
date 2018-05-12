#ifndef RDK2_MODULE_SIMPLEMAPPERMODULE
#define RDK2_MODULE_SIMPLEMAPPERMODULE

#include <vector>

#include <rdkcore/modules/module.h>

#include <rdkcore/sensordata/laserdata.h>
#include <rdkcore/rmaps/ritemonmapvector.h>
#include <rdkcore/rgraphics/rimage.h> // for PIXEL8
#include <rdkcore/rgraphics/rc8set.h>

namespace RDK2 { namespace RAgent {

using namespace RDK2::RGraphics;

class SimpleMapperModule : public Module {
public:

	virtual ~SimpleMapperModule() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	enum Mode { USE_OBJECTS, USE_QUEUE };

private:
	void initMap(Session* session) throw (SessionException);
	void updateMap(Session* session, const Point2od& pose, const RDK2::SensorData::LaserData& rld, const RC8Set* dontTouchColors);

	inline void updatePixel(RImage* img, const RC8Set* dontTouchColors, int x, int y, unsigned char color, vector<Point2i>& modifiedPoints) {
		if (x < 0 || y < 0 || (size_t) x >= img->getWidth() || (size_t) y >= img->getHeight()) return;
		unsigned char* buffer = img->getBuffer();
		if (dontTouchColors->contains(buffer[y * img->getWidth() + x])) return;
		if (buffer[y * img->getWidth() + x] == color) return;
		buffer[y * img->getWidth() + x] = color;
		modifiedPoints.push_back(Point2i(x, y));
	}
};

}} // namespace

#endif
