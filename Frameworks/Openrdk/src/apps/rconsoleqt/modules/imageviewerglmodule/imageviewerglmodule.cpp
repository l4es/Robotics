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

#include "imageviewerglmodule.h"

#include "../../rqcommon/rqglviewer.h"

#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/rgraphics/rimagediffrect.h>
#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "ImageViewerGlModule"

#include <qlabel.h>
#include <qtabwidget.h>
#include <qgroupbox.h>

namespace RDK2 { namespace RConsoleQt {

using namespace RDK2::RMaps;
using namespace RDK2::RGraphics;

bool ImageViewerGlModule::initConfigurationProperties()
{
	if (!BigPropertyViewerModule::initConfigurationProperties()) return false;
	
	SESSION_TRY_START(session)
		session->createVector<RString>(PROPERTY_IMAGE_URLS, "Shown images");
		session->setPersistent(PROPERTY_IMAGE_URLS);
		session->createVector<RString>(PROPERTY_DRAWABLE_URLS, "Shown properties");
		session->setPersistent(PROPERTY_DRAWABLE_URLS);
		session->createDouble(PROPERTY_REFRESH_RATE, "Refresh rate", RDouble::MS, 500.);
		session->createBool(PROPERTY_CLONE_IMAGE, "Clone image while refreshing, instead of locking it", false);
		session->createInt(PROPERTY_EDIT_PEN_SIZE, "Size of the editing pen", 2);
		session->createInt(PROPERTY_EDIT_PEN_COLOR, "Color of the editing pen", RImage::C8Black);
		session->createPose3(PROPERTY_LOOK_AT_POSE, "Look-at pose", Point3od(0, 0, 0, 0, 0, 0));
		session->createInt(PROPERTY_SCALE, "Viewer scale", 100);
	SESSION_END_CATCH_TERMINATE(session)
	return true;
}

bool ImageViewerGlModule::init()
{
	if (!BigPropertyViewerModule::init()) return false;
	
	SESSION_TRY_START(session)
	if (session->getString(PROPERTY_URL) != "") {
		// this widget has been called right now:
		// initialize properties to their defaults, given the url
		Url url = session->getString(PROPERTY_URL);
		Vector<RString>* v;
		session->lock(PROPERTY_IMAGE_URLS, HERE);
		v = session->getObjectAsL<Vector<RString> >(PROPERTY_IMAGE_URLS);
		v->push_back(new RString(url + "|100"));
		session->unlock(PROPERTY_IMAGE_URLS);
		
		string prefix = url.getHost();
		if (prefix == session->getRepositoryName()) prefix = "";
		if (prefix != "") prefix = "rdk://" + prefix;
		
		session->setString(PROPERTY_URL, "");
	}
	
	// FIXME this should be done also when a new map is added, and unsubscribe when a map is removed from the viewer		
	session->lock(PROPERTY_IMAGE_URLS, HERE);
	Vector<RString>* v = session->getObjectAsL<Vector<RString> >(PROPERTY_IMAGE_URLS);
	for (size_t i = 0; i < v->size(); i++) {
		vector<string> t = TextUtils::tokenize(v->getItem(i)->value, "|");
		if (t.size() != 2) {
			RDK_ERROR_PRINTF("Malformed image information (%s)", v->getItem(i)->value.c_str());
		}
		else {
			string url = (t[0][0] == '#' ? t[0].substr(1) : t[0]);
			session->storageSubscribeDiffs(url);
		}
	}
	session->unlock(PROPERTY_IMAGE_URLS);
	
	// FIXME storage subscribing also for all properties
	
	SESSION_END(session)
	return true;
	SESSION_CATCH_TERMINATE(session)
	return false;
}

void ImageViewerGlModule::exec()
{
	WIDGET_GUARD
	
	while (session->wait(), !exiting) {
		SESSION_TRY_START(session)
		session->lock(PROPERTY_IMAGE_URLS, HERE);
		Vector<RString>* imageUrls = session->getObjectAsL<Vector<RString> >(PROPERTY_IMAGE_URLS);
		for (size_t mi = 0; mi < imageUrls->size(); mi++) {
			vector<string> t = TextUtils::tokenize(imageUrls->getItem(mi)->value, "|");
			if (t.size() != 2) {
				RDK_ERROR_PRINTF("Malformed image information (%s)", imageUrls->getItem(mi)->value.c_str());
			}
			else {
				string url = t[0];
				if (url[0] != '#') {
					vector<const RDK2::Object*> v = session->diffsFreeze(url);
					int left = INT_MAX, top = INT_MAX, right = INT_MAX, bottom = INT_MAX;
					for (size_t i = 0; i < v.size(); i++) {
						const RImageDiffRect* rdr = dynamic_cast<const RImageDiffRect*>(v[i]);
						if (rdr) {
							int rl = rdr->rectLeft;
							int rt = rdr->rectTop;
							int rr = rl + rdr->rectWidth;
							int rb = rt + rdr->rectHeight;
							left = (left == INT_MAX ? rl : std::min(left, rl));
							top  = (top == INT_MAX ? rt : std::min(top, rt));
							right = (right == INT_MAX ? rr : std::max(right, rr));
							bottom = (bottom == INT_MAX ? rb : std::max(bottom, rb));
						}
					}
					if (left != INT_MAX && top != INT_MAX && right != INT_MAX && bottom != INT_MAX) {
						((RqGlViewer*)viewerWidget)->declareRectToUpdate(url, left, top, right - left, bottom - top);
					}
				}
			}
		}
		session->unlock(PROPERTY_IMAGE_URLS);
		SESSION_END_CATCH_TERMINATE(session)
	}
}

void ImageViewerGlModule::refreshViewer()
{
	QT_THREAD_GUARD()
		
	RqGlViewer* viewer = (RqGlViewer*) viewerWidget;

	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
		bool cloneImage = guiSession->getBool(PROPERTY_CLONE_IMAGE);
		guiSession->lock(PROPERTY_IMAGE_URLS, HERE);
		Vector<RString>* mapUrls = guiSession->getObjectAsL<Vector<RString> >(PROPERTY_IMAGE_URLS);
		for (size_t i = 0; i < mapUrls->size(); i++) {
			string mu = mapUrls->getItem(i)->value;
			vector<string> t = TextUtils::tokenize(mu, "|");
			if (t.size() != 2) {
				RDK_ERROR_PRINTF("Malformed image information (%s)", mu.size());
			}
			else {
				mu = t[0];
				if (mu[0] == '#') viewer->removeRImage(mu.substr(1));
				else {
					RImage* m = 0;
					if (cloneImage) { m = guiSession->getObjectCloneAs<RImage>(mu); }
					else { guiSession->lock(mu, HERE); m = guiSession->getObjectAsL<RImage>(mu); }
					if (m) viewer->refreshRImage(mu, m);
					viewer->setRImageAlpha(mu, (float) atoi(t[1].c_str()) / 100.);
					if (cloneImage) delete m;
					else guiSession->unlock(mu);
				}
			}
		}
		guiSession->unlock(PROPERTY_IMAGE_URLS);
				
		viewer->clearDrawables();
		guiSession->lock(PROPERTY_DRAWABLE_URLS, HERE);
		Vector<RString>* v = guiSession->getObjectAsL<Vector<RString> >(PROPERTY_DRAWABLE_URLS);
		for (size_t i = 0; i < v->size(); i++) {
			if (v->getItem(i)->value[0] != '#') viewer->addDrawable(v->getItem(i)->value);
		}
		guiSession->unlock(PROPERTY_DRAWABLE_URLS);
		
		//viewer->setScale((double) guiSession->getInt(PROPERTY_SCALE) / 100);
		viewer->setScale((double) currentScale / 100.);
		viewer->update();
	SESSION_END_CATCH_TERMINATE(guiSession)
}

void ImageViewerGlModule::saveOptionsInConfig()
{
	QT_THREAD_GUARD()
	
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
	switch (cboRefreshRate->currentItem()) {
		case CBO_NO_REFRESH: guiSession->setDouble(PROPERTY_REFRESH_RATE, 0.); break;
		case CBO_REFRESH_HALF_FPS: guiSession->setDouble(PROPERTY_REFRESH_RATE, 2000.); break;
		case CBO_REFRESH_1_FPS: guiSession->setDouble(PROPERTY_REFRESH_RATE, 1000.); break;
		case CBO_REFRESH_2_FPS: guiSession->setDouble(PROPERTY_REFRESH_RATE, 500.); break;
		case CBO_REFRESH_5_FPS: guiSession->setDouble(PROPERTY_REFRESH_RATE, 200.); break;
		case CBO_REFRESH_10_FPS: guiSession->setDouble(PROPERTY_REFRESH_RATE, 100.); break;
		default: guiSession->setDouble(PROPERTY_REFRESH_RATE, 0.); break;
	}
	switch (cboEditPenSize->currentItem()) {
		case CBO_PEN_TINY: guiSession->setInt(PROPERTY_EDIT_PEN_SIZE, 1); break;
		case CBO_PEN_SMALL: guiSession->setInt(PROPERTY_EDIT_PEN_SIZE, 2); break;
		case CBO_PEN_MEDIUM: guiSession->setInt(PROPERTY_EDIT_PEN_SIZE, 5); break;
		case CBO_PEN_BIG: guiSession->setInt(PROPERTY_EDIT_PEN_SIZE, 10); break;
		default: guiSession->setInt(PROPERTY_EDIT_PEN_SIZE, 2); break;
	}
	switch (lboxEditPenColor->currentItem()) {
		case CBO_COLOR_BLACK: guiSession->setInt(PROPERTY_EDIT_PEN_COLOR, RImage::C8Black); break;
		case CBO_COLOR_WHITE: guiSession->setInt(PROPERTY_EDIT_PEN_COLOR, RImage::C8White); break;
		case CBO_COLOR_BLUE: guiSession->setInt(PROPERTY_EDIT_PEN_COLOR, RImage::C8Blue); break;
		case CBO_COLOR_RED: guiSession->setInt(PROPERTY_EDIT_PEN_COLOR, RImage::C8Red); break;
		case CBO_COLOR_GREEN: guiSession->setInt(PROPERTY_EDIT_PEN_COLOR, RImage::C8Green); break;
		case CBO_COLOR_MAGENTA: guiSession->setInt(PROPERTY_EDIT_PEN_COLOR, RImage::C8Magenta); break;
		case CBO_COLOR_CYAN: guiSession->setInt(PROPERTY_EDIT_PEN_COLOR, RImage::C8Cyan); break;
		case CBO_COLOR_GRAY: guiSession->setInt(PROPERTY_EDIT_PEN_COLOR, RImage::C8Grey); break;
	}
	guiSession->setBool(PROPERTY_CLONE_IMAGE, chkCloneImage->isChecked());
		
	guiSession->lock(PROPERTY_IMAGE_URLS, HERE);
	Vector<RString>* mapUrls = guiSession->getObjectAsL<Vector<RString> >(PROPERTY_IMAGE_URLS);
	mapUrls->clear();
	for (QListViewItemIterator it(lboxShownMaps); it.current(); ++it) {
		QCheckListItem* itm = dynamic_cast<QCheckListItem*>(it.current());
		if (itm) {
			mapUrls->push_back(new RString(QString("%1|%2")
				.arg(QString(itm->isOn() ? "" : "#") + it.current()->text(0))
				.arg(it.current()->text(1)).latin1()));
		}
	}
	guiSession->unlock(PROPERTY_IMAGE_URLS);
	
	guiSession->lock(PROPERTY_DRAWABLE_URLS, HERE);
	Vector<RString>* drawableUrls = guiSession->getObjectAsL<Vector<RString> >(PROPERTY_DRAWABLE_URLS);
	drawableUrls->clear();
	for (QListViewItemIterator it(lboxShownProps); it.current(); ++it) {
		QCheckListItem* itm = dynamic_cast<QCheckListItem*>(it.current());
		if (itm) {
			string s = QString(itm->isOn() ? "" : "#") + itm->text(0);
			drawableUrls->push_back(new RString(s));
		}
	}
	guiSession->unlock(PROPERTY_DRAWABLE_URLS);
	
	guiSession->setInt(PROPERTY_SCALE, sliderImageScale->value());
	SESSION_END_CATCH_TERMINATE(guiSession)
}

void ImageViewerGlModule::updateSidePanelWithConfig()
{
	QT_THREAD_GUARD()
	
	vector<string> shownMaps, shownPoses, shownProps;
	int scale = 100;
	string editingPose;
	
	Session* guiSession = RqCommon::getGuiSession(getModuleName());
	SESSION_TRY_START(guiSession)
	
	double refreshRate = guiSession->getDouble(PROPERTY_REFRESH_RATE);
	if (refreshRate == 0.) cboRefreshRate->setCurrentItem(CBO_NO_REFRESH);
	else if (refreshRate <= 100.) cboRefreshRate->setCurrentItem(CBO_REFRESH_10_FPS);
	else if (refreshRate <= 200.) cboRefreshRate->setCurrentItem(CBO_REFRESH_5_FPS);
	else if (refreshRate <= 500.) cboRefreshRate->setCurrentItem(CBO_REFRESH_2_FPS);
	else if (refreshRate <= 1000.) cboRefreshRate->setCurrentItem(CBO_REFRESH_1_FPS);
	else if (refreshRate <= 2000.) cboRefreshRate->setCurrentItem(CBO_REFRESH_HALF_FPS);
	else cboRefreshRate->setCurrentItem(CBO_NO_REFRESH);
	slot_cboRefreshRate_activated(cboRefreshRate->currentItem());
	
	switch (guiSession->getInt(PROPERTY_EDIT_PEN_SIZE)) {
		case 1: cboEditPenSize->setCurrentItem(CBO_PEN_TINY); break;
		case 2: cboEditPenSize->setCurrentItem(CBO_PEN_SMALL); break;
		case 5: cboEditPenSize->setCurrentItem(CBO_PEN_MEDIUM); break;
		case 10: cboEditPenSize->setCurrentItem(CBO_PEN_BIG); break;
		default: cboEditPenSize->setCurrentItem(CBO_PEN_SMALL); break;
	}
	
	switch (guiSession->getInt(PROPERTY_EDIT_PEN_COLOR)) {
		case RImage::C8Black: lboxEditPenColor->setCurrentItem(CBO_COLOR_BLACK); break;
		case RImage::C8White: lboxEditPenColor->setCurrentItem(CBO_COLOR_WHITE); break;
		case RImage::C8Blue: lboxEditPenColor->setCurrentItem(CBO_COLOR_BLUE); break;
		case RImage::C8Red: lboxEditPenColor->setCurrentItem(CBO_COLOR_RED); break;
		case RImage::C8Green: lboxEditPenColor->setCurrentItem(CBO_COLOR_GREEN); break;
		case RImage::C8Magenta: lboxEditPenColor->setCurrentItem(CBO_COLOR_MAGENTA); break;
		case RImage::C8Cyan: lboxEditPenColor->setCurrentItem(CBO_COLOR_CYAN); break;
		case RImage::C8Grey: lboxEditPenColor->setCurrentItem(CBO_COLOR_GRAY); break;
		default: lboxEditPenColor->setCurrentItem(CBO_COLOR_BLACK); break;
	}
	
	guiSession->lock(PROPERTY_IMAGE_URLS, HERE);
	Vector<RString>* mapUrls = guiSession->getObjectAsL<Vector<RString> >(PROPERTY_IMAGE_URLS);
	for (size_t i = 0; i < mapUrls->size(); i++) {
		shownMaps.push_back(mapUrls->getItem(i)->value);
	}
	guiSession->unlock(PROPERTY_IMAGE_URLS);
	
	guiSession->lock(PROPERTY_DRAWABLE_URLS, HERE);
	Vector<RString>* drawableUrls = guiSession->getObjectAsL<Vector<RString> >(PROPERTY_DRAWABLE_URLS);
	for (size_t i = 0; i < drawableUrls->size(); i++) {
		shownProps.push_back(drawableUrls->getItem(i)->value);
	}
	guiSession->unlock(PROPERTY_DRAWABLE_URLS);
	
	chkCloneImage->setChecked(guiSession->getBool(PROPERTY_CLONE_IMAGE));
	
	scale = guiSession->getInt(PROPERTY_SCALE);
	SESSION_END_CATCH_TERMINATE(guiSession)

	slot_cboRefreshRate_activated(cboRefreshRate->currentItem());
	
	//QListViewItem* itm = lboxShownMaps->currentItem();
	int oldidx = cboEditTargetMap->currentItem();
	string oldSelection = "";
	lboxShownMaps->clear();
	cboEditTargetMap->clear();
	for (size_t i = 0; i < shownMaps.size(); i++) {
		vector<string> v = TextUtils::tokenize(shownMaps[i], "|");
		if (v.size() != 2) {
			RDK_ERROR_PRINTF("Malformed image information (%s)", shownMaps[i].c_str());
		}
		else {
			bool selected = true;
			if (v[0].size() > 1 && v[0][0] == '#') { selected = false; v[0] = v[0].substr(1); }
			QCheckListItem* itm = new QCheckListItem(lboxShownMaps, v[0], QCheckListItem::CheckBox);
			itm->setOn(selected);
			itm->setText(1, v[1]);
			cboEditTargetMap->insertItem(v[0]);
		}
	}
	if (oldSelection != "") lboxShownMaps->setSelected(lboxShownMaps->findItem(oldSelection, 0), true);
	if (oldidx != -1) cboEditTargetMap->setCurrentItem(oldidx);
	
	for (size_t i = 0; i < shownProps.size(); i++) {
		vector<string> v = TextUtils::tokenize(shownProps[i], "|");
		if (v.size() != 1) {
			RDK_ERROR_PRINTF("Malformed drawable property information (%s)", shownProps[i].c_str());
		}
		else {
			bool selected = true;
			if (v[0].size() > 1 && v[0][0] == '#') { selected = false; v[0] = v[0].substr(1); }
			QCheckListItem* itm = new QCheckListItem(lboxShownProps, v[0], QCheckListItem::CheckBox);
			itm->setOn(selected);
		}
	}
	
	QListBoxItem* etm = lboxEditablePoses->findItem(editingPose);
	if (etm) lboxEditablePoses->setCurrentItem(etm);
	
	lboxEditablePoses->setCurrentItem(lboxEditablePoses->findItem(oldSelection));
	sliderImageScale->setValue(scale);
	currentScale = scale;
}

MODULE_FACTORY(ImageViewerGlModule);

}} // ns
