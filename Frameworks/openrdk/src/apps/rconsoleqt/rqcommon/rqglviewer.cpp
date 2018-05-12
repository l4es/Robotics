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

#include "rqglviewer.h"
#include "rqcommon.h"

#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/rmaps/rline2donmap.h>
#include <rdkcore/rmaps/rpoint2donmap.h>
#include <rdkcore/rmaps/rellipseonmap.h>
#include <rdkcore/rmaps/rarrowonmap.h>
#include <rdkcore/rmaps/rlabelonmap.h>
#include <rdkcore/geometry/robotmodels.h>
#include <rdkcore/geometry/point2.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "RqGlViewer"

#include <qcursor.h>
#include <qimage.h>
#include <qpainter.h>

// 1m = 100 pixels
#define WIDGET_GL_SCALE 100.0

#include <iostream>
using std::cerr;
using std::endl;

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::TextUtils;
using namespace RDK2::RAgent;
using namespace RDK2::RMaps;

RqGlViewer::RqGlViewer(QWidget* parent) :
	QGLWidget(parent), alwaysCenteredPoseTheta(false), cx(0.), cy(0.), pan(0.), tilt(0.), roll(0.), scale(1.),
	dragging(false), cx0(0.), cy0(0.), pan0(0.), tilt0(0.), roll0(0.), scale0(1.)
{
	setMouseTracking(true);
	setCursor(QCursor(Qt::CrossCursor));
	usleep(300000);		// XXX if everything goes up at once, it seems that there is some problem with multiple GL contexts
}

RqGlViewer::~RqGlViewer()
{
	for (map<string, ShownMap>::iterator it = shownMaps.begin(); it != shownMaps.end(); ++it) {
		delete it->second.imgToGlTex;
	}
}

void RqGlViewer::refreshRMapImage(const string& url, RMapImage* m)
{
	bool exiting = false; QT_THREAD_GUARD()
	makeCurrent();
	
	rectsToUpdateMutex.lock(HERE);
	map<string, ShownMap>::iterator it = shownMaps.find(url);
	if (it != shownMaps.end()) {
		map<string, Rect>::iterator updit = rectsToUpdate.find(url);
		if (updit == rectsToUpdate.end()) it->second.imgToGlTex->updateTexture(m);
		else it->second.imgToGlTex->updateTexture(m, updit->second.x, updit->second.y, updit->second.w, updit->second.h);
	}
	else {
		RImageToGlTexture* imgtex = new RImageToGlTexture();
		imgtex->updateTexture(m);
		ShownMap sm;
		sm.imgToGlTex = imgtex;
		shownMaps.insert(make_pair(url, sm));
	}
	rectsToUpdate.clear();
	rectsToUpdateMutex.unlock();
}

void RqGlViewer::refreshRImage(const string& url, RImage* img)
{
	bool exiting = false; QT_THREAD_GUARD()
	makeCurrent();
	
	rectsToUpdateMutex.lock(HERE);
	map<string, ShownImage>::iterator it = shownImages.find(url);
	if (it != shownImages.end()) {
		map<string, Rect>::iterator updit = rectsToUpdate.find(url);
		if (updit == rectsToUpdate.end()) it->second.imgToGlTex->updateTexture(img);
		else it->second.imgToGlTex->updateTexture(img, updit->second.x, updit->second.y, updit->second.w, updit->second.h);
	}
	else {
		RImageToGlTexture* imgtex = new RImageToGlTexture();
		imgtex->updateTexture(img);
		ShownImage simg;
		simg.imgToGlTex = imgtex;
		shownImages.insert(make_pair(url, simg));
	}
	rectsToUpdate.clear();
	rectsToUpdateMutex.unlock();
}

void RqGlViewer::removeRMapImage(const string& url)
{
	bool exiting = false; QT_THREAD_GUARD()

	map<string, ShownMap>::iterator it = shownMaps.find(url);
	if (it != shownMaps.end()) {
		delete it->second.imgToGlTex;
		shownMaps.erase(it);
	}
}

void RqGlViewer::removeRImage(const string& url)
{
	bool exiting = false; QT_THREAD_GUARD()

	map<string, ShownImage>::iterator it = shownImages.find(url);
	if (it != shownImages.end()) {
		delete it->second.imgToGlTex;
		shownImages.erase(it);
	}
}

void RqGlViewer::setRMapImageAlpha(const string& url, double alpha)
{
	bool exiting = false; QT_THREAD_GUARD()
	map<string, ShownMap>::iterator it = shownMaps.find(url);
	if (it != shownMaps.end()) {
		it->second.alpha = alpha;
	}
}

void RqGlViewer::setRImageAlpha(const string& url, double alpha)
{
	bool exiting = false; QT_THREAD_GUARD()
	map<string, ShownImage>::iterator it = shownImages.find(url);
	if (it != shownImages.end()) {
		it->second.alpha = alpha;
	}
}

void RqGlViewer::addPose(const string& url)
{
	bool exiting = false; QT_THREAD_GUARD()
	vector<string> v = tokenize(url, "|");
	if (v.size() == 3) {
		v.insert(++(v.begin()), "F");
	}
	if (v.size() != 4) {
		RDK_ERROR_PRINTF("Malformed url passed to RqGlViewer (%s)", url.c_str());
		return;
	}

	vector<string> c = tokenize(v[2], ",");
	if (c.size() != 3) {
		RDK_ERROR_PRINTF("Malformed url passed to RqGlViewer (%s)", url.c_str());
		return;
	}
	
	ShownPose& sp = shownPoses[v[0]];
	sp.robotPrefix = v[3];
	sp.r = (float) atoi(c[0].c_str()) / 255.;
	sp.g = (float) atoi(c[1].c_str()) / 255.;
	sp.b = (float) atoi(c[2].c_str()) / 255.;
	sp.accumulate = (v[1] == "T");
}

void RqGlViewer::clearPose(const string& url)
{
	bool exiting = false; QT_THREAD_GUARD()
	string purl = url.substr((url[0] == '#'),url.find_first_of('|')-1);
	shownPoses[purl].poses.clear();
	shownPoses.erase(purl);
}

void RqGlViewer::addDrawable(const string& url)
{
	bool exiting = false; QT_THREAD_GUARD()
	shownDrawables.insert(url);
}

void RqGlViewer::clearDrawables()
{
	bool exiting = false; QT_THREAD_GUARD()
	shownDrawables.clear();
}

void RqGlViewer::setAlwaysCenteredPose(const string& url, bool alsoTheta)
{
	bool exiting = false; QT_THREAD_GUARD()
	alwaysCenteredPose = url;
	alwaysCenteredPoseTheta = alsoTheta;
}

void RqGlViewer::declareRectToUpdate(const string& url, int x, int y, int w, int h)
{
	rectsToUpdateMutex.lock(HERE);
	map<string, Rect>::iterator it = rectsToUpdate.find(url);
	if (it == rectsToUpdate.end()) {
		Rect r; r.x = x; r.y = y; r.w = w; r.h = h;
		rectsToUpdate.insert(make_pair(url, r));
	}
	else {
		Rect& oldr = it->second;
		Rect r;
		r.x = std::min(oldr.x, x);
		r.y = std::min(oldr.y, y);
		r.w = (oldr.x + oldr.w > r.x + w ? oldr.x + oldr.w - r.x : w);
		r.h = (oldr.y + oldr.h > r.y + h ? oldr.y + oldr.h - r.y : h);
		it->second = r;
	}
	rectsToUpdateMutex.unlock();
}

void RqGlViewer::paintGL()
{
	bool exiting = false; QT_THREAD_GUARD()
	glClear(GL_COLOR_BUFFER_BIT);

	glLoadIdentity();

	if (alwaysCenteredPose == "IMAGE") {
		cx = 0.0; cy = 0.0;
		pan = 0.0; tilt = 0.0; roll = 0.0;
	}
	if (alwaysCenteredPose != "") {
		Point2od curPose;
		Session* guiSession = RqCommon::getGuiSession("");
		SESSION_TRY_START(guiSession)
		curPose = guiSession->getPose(alwaysCenteredPose);
		cx = curPose.x; cy = curPose.y;
		if (alwaysCenteredPoseTheta) roll = curPose.theta;
		SESSION_END_CATCH_TERMINATE(guiSession);
	}
	
	glScalef(scale, scale, 1.0);
	
	gluLookAt(10.0 * sin(pan) + cx, 10.0 * sin(tilt) + cy, 10.0 * cos(pan) * cos(tilt),
				cx, cy, 0.0,
				1.0 * sin(roll), 1.0 * cos(roll), 0.0);

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	//glEnable(GL_BLEND);
	glDisable(GL_DEPTH_TEST);
	glColor3f(1.0, 1.0, 1.0);

	glEnable(GL_TEXTURE_2D);
	for (map<string, ShownMap>::iterator it = shownMaps.begin(); it != shownMaps.end(); ++it) {
		glColor4f(1.0, 1.0, 1.0, it->second.alpha);
		it->second.imgToGlTex->drawMapQuad();
	}
	glScalef(0.02, 0.02, 1.);
	for (map<string, ShownImage>::iterator it = shownImages.begin(); it != shownImages.end(); ++it) {
		glColor4f(1.0, 1.0, 1.0, it->second.alpha);
		it->second.imgToGlTex->drawMapQuad();
	}
	glScalef(50., 50., 1.);
	glDisable(GL_TEXTURE_2D);

	glDisable(GL_COLOR_MATERIAL);
	//glDisable(GL_BLEND);
	//glEnable(GL_DEPTH_TEST);
	drawPoses();
	drawDrawables();


	swapBuffers();

	//// Make GL Context current
	//makeCurrent();
	//// Copy from OpenGL
	//QImage *tempImage = new QImage( grabFrameBuffer() );
	//static int counter=0;
	//++counter;
	//ostringstream oss;
	//oss << "/tmp/rq" << counter <<  ".jpg";
	//if ( ! tempImage || !tempImage->save( oss.str().c_str(), "JPEG") )
	//  cerr << "ERRORRRRRRR";
	//// Cleaning memory
	//delete tempImage;

//  if (true)
//  {
//    GLint viewport[4];
//    GLdouble modelview[16];
//    GLdouble projection[16];
//    GLfloat winX, winY;

//    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
//    glGetDoublev(GL_PROJECTION_MATRIX, projection);
//    glGetIntegerv(GL_VIEWPORT, viewport);

//    winX = (float) 0;
//    winY = (float) viewport[3] - (float) 0;
////		glReadPixels((int) winX, (int) winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
//    unsigned char* imagetodump = new unsigned char[viewport[2] * viewport[3] * 3];
//    glReadPixels(viewport[0], viewport[1], viewport[2], viewport[3], GL_RGBA, GL_UNSIGNED_BYTE, imagetodump);

//    static int counter=0;
//    ostringstream oss;
//    oss  << "--->"            << viewport[2] << " " << viewport[3];
//    cerr << oss.str().c_str() << endl ;
//    Magick::Image img(viewport[2],viewport[3],"RGBA",Magick::CharPixel,imagetodump);
//    img.backgroundColor(img.pixelColor(0,0));
//    img.matte(false);
//    img.page(Magick::Geometry(0,0));
//    ostringstream filename;
//    filename << "fn-" << counter++ << ".ppm";
//    cerr << filename.str() << endl;
//    img.write(filename.str());
//    delete imagetodump;
//  }

}

void RqGlViewer::drawPoses()
{
	bool exiting = false; QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession("");
	float z = -0.10;
	SESSION_TRY_START(guiSession)
	for (map<string, ShownPose>::iterator it = shownPoses.begin(); it != shownPoses.end(); ++it)
	{
		vector<Point2od>& vposes = it->second.poses;
		string purl = it->first;

		string robotPrefix = it->second.robotPrefix;

		if (!it->second.accumulate)
			vposes.clear();
		vposes.push_back(guiSession->getPose(purl));

		glColor3f(it->second.r, it->second.g, it->second.b);
		for (vector<Point2od>::const_iterator vit=vposes.begin();
			   vit!=vposes.end();
				 ++vit)
		{
			glPushMatrix();
			const Point2od& pose = *vit;
			glTranslatef(pose.x, pose.y, z);

			if (fabs(pose.theta) > 2 * M_PI) {
				// undefined orientation: draw a cross
				double radius = guiSession->getDouble(robotPrefix + PROPERTY_ROBOT_RADIUS);
				glLineWidth(4.0);
				glBegin(GL_LINES);
					glVertex3f(-radius/2, -radius/2, z);
					glVertex3f(radius/2, radius/2, z);
					glVertex3f(-radius/2, radius/2, z);
					glVertex3f(radius/2, -radius/2, z);
				glEnd();
			}
			else {
				glRotatef(pose.theta * 180 / M_PI, 0.0, 0.0, 1.0);	// OpenGL uses degrees, not radians
				RDK2::Common::RobotShape shape = (RDK2::Common::RobotShape) guiSession->getInt(robotPrefix + PROPERTY_ROBOT_SHAPE);
				switch (shape) {
					case (RDK2::Common::CIRCULAR): {
						double radius = guiSession->getDouble(robotPrefix + PROPERTY_ROBOT_RADIUS);
						glLineWidth(2.0);
						glBegin(GL_LINES);
							glVertex3f(0.0, 0.0, z);
							glVertex3f(radius, 0.0, z);
						glEnd();
						glBegin(GL_LINE_LOOP);
						for (int i = 0; i < 10; i++) {
							double a = (double) i / 10 * M_PI * 2;
							glVertex3f(cos(a) * radius, sin(a) * radius, z);
						}
						glEnd();
						break;
					}
				case (RDK2::Common::RECTANGULAR): {
						double w = guiSession->getDouble(robotPrefix + PROPERTY_ROBOT_WIDTH);
						double h = guiSession->getDouble(robotPrefix + PROPERTY_ROBOT_HEIGHT);
						glLineWidth(2.0);
						glBegin(GL_LINE_STRIP);
							glVertex3f(0.0, 0.0, z);
							glVertex3f(h/2, 0.0, z);
							glVertex3f(h/2, w/2, z);
							glVertex3f(-h/2, w/2, z);
							glVertex3f(-h/2, -w/2, z);
							glVertex3f(h/2, -w/2, z);
							glVertex3f(h/2, 0.0, z);
						glEnd();
						break;
					}
				}
			}
			glPopMatrix();
		}
		z -= 0.01;
	}
	SESSION_END_CATCH_TERMINATE(guiSession)
}

void RqGlViewer::drawDrawables()
{
	bool exiting = false; QT_THREAD_GUARD()
	Session* guiSession = RqCommon::getGuiSession("");
	for (set<string>::iterator it = shownDrawables.begin(); it != shownDrawables.end(); ++it) {
		SESSION_TRY_START(guiSession)
		string propertyName = *it;
		
		int was = 0;	// 0 = first time, 1 = points, 2 = lines
		
		guiSession->lock(propertyName, HERE);
		Vector<RItemOnMap>* v = guiSession->getObjectAsL<Vector<RItemOnMap> >(propertyName);
		for (Vector<RItemOnMap>::iterator it = v->begin(); it != v->end(); ++it) {
			RLine2dOnMap* line = dynamic_cast<RLine2dOnMap*>(*it);
			if (line) {
				float r = (float) rgbGetRed(line->color) / 256;
				float g = (float) rgbGetGreen(line->color) / 256;
				float b = (float) rgbGetBlue(line->color) / 256;
				glColor3f(r, g, b);
				//glLineWidth((float) 2.0/*line->drawWidth*/);    // FIXME questa non l'ho proprio capita
				glLineWidth((float) line->drawWidth);
				//if (line->drawWidth != 2.0) RDK_DEBUG_PRINTF("%.2f", line->drawWidth);
				glBegin(GL_LINES);
					glVertex3f(line->p0.x, line->p0.y, 4.5);
					glVertex3f(line->p1.x, line->p1.y, 4.5);
				glEnd();
				was = 2;
				continue;
			}

			RPoint2dOnMap* point = dynamic_cast<RPoint2dOnMap*>(*it);
			if (point) {
				float r = (float) rgbGetRed(point->color) / 256;
				float g = (float) rgbGetGreen(point->color) / 256;
				float b = (float) rgbGetBlue(point->color) / 256;
				glColor3f(r, g, b);
				glPointSize((float) point->drawWidth);

				glBegin(GL_POINTS);
					glVertex3f(point->x, point->y, 4.5);
				glEnd();
				was = 1;
				continue;
			}

			REllipseOnMap* ellipse = dynamic_cast<REllipseOnMap*>(*it);
			if (ellipse) {
				float r = (float) rgbGetRed(ellipse->color) / 256;
				float g = (float) rgbGetGreen(ellipse->color) / 256;
				float b = (float) rgbGetBlue(ellipse->color) / 256;
				glColor3f(r, g, b);
				glPointSize((float) ellipse->drawWidth);


				float TWOPI = M_PI*2;
				size_t n    = 20;
				float dth   = TWOPI/n;
				if (ellipse->filled)
				{
					glBegin(GL_POLYGON);
				}
				else
				{
					glBegin(GL_LINE_LOOP);
				}
				for(float t = 0; t <= TWOPI; t += dth)
				{
					RDK2::Geometry::Point2od p = add(static_cast<RDK2::Geometry::Point2od>(*ellipse),RDK2::Geometry::Point2od(ellipse->a*cos(t),ellipse->b*sin(t),0.));
					glVertex3f(p.x,p.y,4.5);
				}
				glEnd();
				was = 1;
				continue;
			}

			RArrowOnMap* arrow = dynamic_cast<RArrowOnMap*>(*it);
			if (arrow) {
				float r = (float) rgbGetRed(arrow->color) / 256;
				float g = (float) rgbGetGreen(arrow->color) / 256;
				float b = (float) rgbGetBlue(arrow->color) / 256;
				glColor3f(r, g, b);
				glLineWidth((float) arrow->drawWidth);

				double x1,x2,y1,y2;
				RArrowOnMap::calcVertexes(*arrow,x1,y1,x2,y2);
				double hx = arrow->x + arrow->delta*cos(arrow->theta),
							 hy = arrow->y + arrow->delta*sin(arrow->theta);
				glBegin(GL_LINES);
					glVertex3f(arrow->x, arrow->y, 4.5);
					glVertex3f(hx, hy, 4.5);
				glEnd();
				switch(arrow->type)
				{
					case RArrowOnMap::CIRCLE:
						{
							float TWOPI = M_PI*2;
							size_t n    = 20;
							float dth   = TWOPI/n;
							if (arrow->filled)
							{
								glBegin(GL_POLYGON);
							}
							else
							{
								glBegin(GL_LINE_LOOP);
							}
							Point2od p2od = static_cast<RDK2::Geometry::Point2od>(*arrow);
							for(float t = 0; t <= TWOPI; t += dth)
							{
								RDK2::Geometry::Point2od p = add(p2od,RDK2::Geometry::Point2od(arrow->headLength*cos(t),arrow->headLength*sin(t),0.));
								glVertex3f(p.x,p.y,4.5);
							}
							glEnd();
						}
						break;
					case RArrowOnMap::DIAMOND:
						{
							double x3 = hx - 2 * arrow->headLength * cos(arrow->theta);
							double y3 = hy - 2 * arrow->headLength * sin(arrow->theta);
							if (arrow->filled)
							{
								glBegin(GL_QUADS);
							}
							else
							{
								glBegin(GL_LINE_LOOP);
							}
								glVertex3f(x1,y1,4.5);
								glVertex3f(x3,y3,4.5);
								glVertex3f(x2,y2,4.5);
								glVertex3f(hx,hy,4.5);
							glEnd();
						}
						break;
					case RArrowOnMap::SOLID:
						{
							if (arrow->filled)
							{
								glBegin(GL_TRIANGLES);
							}
							else
							{
								glBegin(GL_LINE_LOOP);
							}
								glVertex3f(x1,y1,4.5);
								glVertex3f(x2,y2,4.5);
								glVertex3f(hx,hy,4.5);
							glEnd();
						}
						break;
					default:
					case RArrowOnMap::OPEN:
						{
							glBegin(GL_LINE_STRIP);
								glVertex3f(x1,y1,4.5);
								glVertex3f(hx,hy,4.5);
								glVertex3f(x2,y2,4.5);
							glEnd();
						}
						break;
				}
				was = 1;
				continue;
			}

			RLabelOnMap* label = dynamic_cast<RLabelOnMap*>(*it);
			if (label) {
				float r = (float) rgbGetRed(label->color) / 256;
				float g = (float) rgbGetGreen(label->color) / 256;
				float b = (float) rgbGetBlue(label->color) / 256;
				glColor3f(r, g, b);
				//glPushMatrix();
				renderText(label->x, label->y, 0, QString(label->text));
				//glPopMatrix();
				was = 1;
				continue;
			}
		}
		was = was;
		guiSession->unlock(propertyName);
		SESSION_END_CATCH_TERMINATE(guiSession)
	}
}

void RqGlViewer::resizeGL(int w, int h)
{
	makeCurrent();
	bool exiting = false; QT_THREAD_GUARD()
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-((float) w / WIDGET_GL_SCALE),
			((float) w / WIDGET_GL_SCALE),
			-((float) h / WIDGET_GL_SCALE),
			((float) h / WIDGET_GL_SCALE),
			5.0, 15.0);
	glMatrixMode(GL_MODELVIEW);
}

void RqGlViewer::mousePressEvent(QMouseEvent* e)
{
	makeCurrent();
	bool exiting = false; QT_THREAD_GUARD()
	Point2d p = widget2world(e->x(), e->y());
	if (e->button() & QMouseEvent::LeftButton) {
		// LEFT MOUSE BUTTON = start dragging
		m0x = e->x(); m0y = e->y();
		pan0 = pan; tilt0 = tilt; roll0 = roll;
		cx0 = cx; cy0 = cy;
		scale0 = scale;
		dragging = true;
	}
	emit signal_mousePressed(p.x, p.y, (Qt::ButtonState) (e->state() | e->button()));
}

void RqGlViewer::mouseMoveEvent(QMouseEvent* e)
{
	makeCurrent();
	bool exiting = false; QT_THREAD_GUARD()
	Point2d p = widget2world(e->x(), e->y());
	if (e->state() & QMouseEvent::LeftButton && !(e->state() & (QMouseEvent::KeyButtonMask))) {
		// LEFT MOUSE BUTTON only = translate
		double ax = (e->x() - m0x) / WIDGET_GL_SCALE * 2 / scale;
		double ay = (e->y() - m0y) / WIDGET_GL_SCALE * 2 / scale;
		cx = cx0 - ax * cos(roll) + ay * sin(roll);
		cy = cy0 + ax * sin(roll) + ay * cos(roll);
		update();
	}
	else if (e->state() & QMouseEvent::LeftButton && e->state() & QMouseEvent::ControlButton) {
		// CTRL + LEFT MOUSE BUTTON = rotate
		int qcx = width() / 2, qcy = height() / 2;
		int x0 = (int) (m0x - qcx), y0 = (int) (m0y - qcy), x1 = (int) (e->x() - qcx), y1 = (int) (e->y() - qcy);
		double a0 = atan2((double) y0, (double) x0), a1 = atan2((double) y1, (double) x1);
		roll = a0 - a1;
		roll = roll0 + roll;
		update();
	}
	emit signal_mouseMoved(p.x, p.y, e->state());
}

void RqGlViewer::mouseReleaseEvent(QMouseEvent* e)
{
	makeCurrent();
	bool exiting = false; QT_THREAD_GUARD()
	Point2d p = widget2world(e->x(), e->y());
	emit signal_mouseReleased(p.x, p.y, (Qt::ButtonState) (e->state() | e->button()));
	dragging = false;
}

void RqGlViewer::wheelEvent(QWheelEvent* e)
{
	makeCurrent();
	bool exiting = false; QT_THREAD_GUARD()
	double s = (double) e->delta() / 120 / 10;	// WHEEL_DELTA, see Qt3 documentation for QWheelEvent (each tick is 10%)
	scale -= s;
	if (scale < 0.01) scale = 0.01;
	if (scale > 10.0) scale = 10.0;
	emit signal_scaleChanged(scale);
}

Point2d RqGlViewer::widget2world(int x, int y)
{
	bool exiting = false; QT_THREAD_GUARD(Point2d(0, 0))
	GLint viewport[4];
	GLdouble modelview[16];
	GLdouble projection[16];
	GLfloat winX, winY, winZ;
	GLdouble posX, posY, posZ;

	glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	winX = (float) x;
	winY = (float) viewport[3] - (float) y;
	glReadPixels((int) winX, (int) winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

	gluUnProject(winX, winY, 5.0, modelview, projection, viewport, &posX, &posY, &posZ);
	
	return Point2d((double) posX, (double) posY);
}

}} // namespaces
