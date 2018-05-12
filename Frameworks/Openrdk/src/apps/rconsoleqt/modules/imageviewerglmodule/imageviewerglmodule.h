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

#ifndef RDK2_RQM_IMAGEVIEWERGLMODULE
#define RDK2_RQM_IMAGEVIEWERGLMODULE

#include "../../rqcommon/bigpropertyviewermodule.h"
#include "../../rqcommon/rqpropertylistview.h"

#include <qlabel.h>
#include <qslider.h>
#include <qcombobox.h>
#include <qcheckbox.h>
#include <qgroupbox.h>
#include <qlistbox.h>
#include <qpushbutton.h>
#include <qlistview.h>
#include <qlistbox.h>

#define LBL_IMAGE_SCALE "Image scale (%1%):"
#define LBL_MAP_ALPHA "Alpha (%1%): "

#define PROPERTY_IMAGE_URLS "params/imageUrls"
#define PROPERTY_DRAWABLE_URLS "params/drawableUrls"
#define PROPERTY_REFRESH_RATE "params/refreshRate"
#define PROPERTY_CLONE_IMAGE "params/cloneImage"

#define PROPERTY_EDIT_PEN_SIZE "params/penSize"
#define PROPERTY_EDIT_PEN_COLOR "params/penColor"

#define PROPERTY_LOOK_AT_POSE "params/lookAtPose"
#define PROPERTY_SCALE "params/scale"

#define CBO_NO_REFRESH       0
#define CBO_REFRESH_HALF_FPS 1
#define CBO_REFRESH_1_FPS    2
#define CBO_REFRESH_2_FPS    3
#define CBO_REFRESH_5_FPS    4
#define CBO_REFRESH_10_FPS   5

#define CBO_PEN_TINY   0
#define CBO_PEN_SMALL  1
#define CBO_PEN_MEDIUM 2
#define CBO_PEN_BIG    3

#define CBO_COLOR_BLACK   0
#define CBO_COLOR_WHITE   1
#define CBO_COLOR_BLUE    2
#define CBO_COLOR_RED     3
#define CBO_COLOR_GREEN   4
#define CBO_COLOR_MAGENTA 5
#define CBO_COLOR_CYAN    6
#define CBO_COLOR_GRAY    7

namespace RDK2 { namespace RConsoleQt {
		
class ImageViewerGlModule: public BigPropertyViewerModule {
Q_OBJECT
public:
	ImageViewerGlModule() : posing(false), timerSaveToConfig(this), currentScale(100) { }
	
	bool initConfigurationProperties();
	bool init();
	void exec();

protected:
	QWidget* createSideWidget(QWidget* parent);
	QWidget* createViewerWidget(QWidget* parent);
	void refreshViewer();
	void saveOptionsInConfig();
	void updateSidePanelWithConfig();
		
	// "General" tab
	QGroupBox* gboxImageScale;
	QSlider* sliderImageScale;
	QComboBox* cboRefreshRate;
	QCheckBox* chkCloneImage;
	QListBox* lboxEditablePoses;
	
	// "Show" tab
	QListView* lboxShownMaps;
	QLabel* lblSelectedMap;
	QLabel* lblMapAlpha;
	QSlider* sliderMapAlpha;
	QPushButton* btnRemoveShownMapSelected;
	QLabel* lblShownMapInfo;
	
	QListView* lboxShownPoses;
	QLabel* lblSelectedPose;
	QPushButton* btnRemoveShownPoseSelected;
	QPushButton* btnChangeColorPoseSelected;
	QLabel* lblShownPoseInfo;
	
	QListView* lboxShownProps;
	
	// "Edit" tab
	QComboBox* cboEditTargetMap;
	QListBox* lboxEditPenColor;
	QComboBox* cboEditPenSize;
	
public slots:
	void slot_sliderImageScale_valueChanged(int);
	void slot_cboRefreshRate_activated(int);
	void slot_btnChangeColorPoseSelected_clicked();

	void slot_sliderMapAlpha_valueChanged(int);
	
	void slot_lboxShownMaps_selectionChanged(QListViewItem*);
	void slot_lboxShownPoses_selectionChanged(QListViewItem*);
	
	void slot_viewerWidget_mousePressed(double, double, Qt::ButtonState);
	void slot_viewerWidget_mouseMoved(double, double, Qt::ButtonState);
	void slot_viewerWidget_mouseReleased(double, double, Qt::ButtonState);
	void slot_viewerWidget_scaleChanged(double);
	
	void slot_timerSaveToConfig_timeout();

protected:	
	bool posing;
	double posingX, posingY;
	
	bool eventFilter(QObject*,QEvent*);
	
	QTimer timerSaveToConfig;

	int currentScale;
};


}} // ns

#endif
