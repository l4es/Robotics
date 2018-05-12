/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi (daniele.calisi@dis.uniroma1.it)
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

#ifndef RDK2_RQ_RQGLVIEWER
#define RDK2_RQ_RQGLVIEWER

#include "rimagetogltexture.h"

#include <rdkcore/rmaps/rmapimage.h>
#include <rdkcore/posixconstructs/posixmutex.h>

#include <qgl.h>

#include <string>
#include <vector>
#include <map>
#include <set>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RMaps;
using namespace PosixConstructs;
using namespace std;

class RqGlViewer : public QGLWidget
{
Q_OBJECT
public:
	RqGlViewer(QWidget* parent);
	~RqGlViewer();

	// note: values of poses and items are updated during each QWidget::update()
	// maps are updated only when refreshRMapImage is called

	void refreshRMapImage(const string& url, RMapImage* m);
	void removeRMapImage(const string& url);
	void setRMapImageAlpha(const string& url, double alpha);
	
	void refreshRImage(const string& url, RImage* img);
	void removeRImage(const string& url);
	void setRImageAlpha(const string& url, double alpha);
	
	void addPose(const string& url);
	void clearPose(const string& url);
	
	void addDrawable(const string& url);
	void clearDrawables();

	void declareRectToUpdate(const string& url, int x, int y, int w, int h);

	void setAlwaysCenteredPose(const string& url, bool alsoTheta);
	inline void setScale(double s) { scale = s; update(); }

protected:
	void mousePressEvent(QMouseEvent* e);
	void mouseMoveEvent(QMouseEvent* e);
	void mouseReleaseEvent(QMouseEvent* e);
	void wheelEvent(QWheelEvent* e);
	
signals:
	void signal_mousePressed(double, double, Qt::ButtonState);
	void signal_mouseReleased(double, double, Qt::ButtonState);
	void signal_mouseMoved(double, double, Qt::ButtonState);
	void signal_scaleChanged(double);

protected:
	struct ShownPose {
		string robotPrefix;
		double r, g, b;
		bool accumulate;
		vector<Point2od> poses;
	};
	
	struct ShownMap {
		ShownMap() : imgToGlTex(0), alpha(1.0) { }
		RImageToGlTexture* imgToGlTex;
		double alpha;
	};
	
	struct ShownImage {
		ShownImage() : imgToGlTex(0), alpha(1.0) { }
		RImageToGlTexture* imgToGlTex;
		double alpha;
	};
	
	struct Rect {
		int x, y, w, h;
	};
	
	map<string, ShownMap> shownMaps;
	map<string, ShownImage> shownImages;
	PosixMutex rectsToUpdateMutex;
	map<string, Rect> rectsToUpdate;
	map<string, ShownPose> shownPoses;
	set<string> shownDrawables;
	string alwaysCenteredPose;
	bool alwaysCenteredPoseTheta;
	
	void paintGL();
	void resizeGL(int w, int h);
	
	void drawPoses();
	void drawDrawables();
	
	Point2d widget2world(int x, int y);
	
	double cx, cy, pan, tilt, roll, scale;
	bool dragging;
	int m0x, m0y;
	double cx0, cy0, pan0, tilt0, roll0, scale0;
	//RImage tosave;
};

}} // namespace

#endif
