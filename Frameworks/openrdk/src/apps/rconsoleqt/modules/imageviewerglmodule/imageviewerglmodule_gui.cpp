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
#include "../../rqcommon/rqpropertylistview.h"

#include <rdkcore/logging/logging.h>
#define LOGGING_MODULE "ImageViewerGlModule(GUI)"

#include <qtabwidget.h>
#include <qgroupbox.h>
#include <qcolordialog.h>
#include <qdragobject.h>

#include <cfloat>

namespace RDK2 { namespace RConsoleQt {

void ImageViewerGlModule::slot_sliderImageScale_valueChanged(int v)
{
	QT_THREAD_GUARD()
	gboxImageScale->setTitle(QString(LBL_IMAGE_SCALE).arg(v));
	if (viewerWidget) {
		((RqGlViewer*)viewerWidget)->setScale((float) v / 100.);
		currentScale = v;
	}
}

void ImageViewerGlModule::slot_sliderMapAlpha_valueChanged(int v)
{
	QT_THREAD_GUARD()
	QListViewItem* itm = lboxShownMaps->currentItem();
	if (itm) {
		itm->setText(1, QString("%1").arg(v));
	}
	lblMapAlpha->setText(QString(LBL_MAP_ALPHA).arg(v));
}

void ImageViewerGlModule::slot_timerSaveToConfig_timeout()
{
	saveOptionsInConfig();
}

void ImageViewerGlModule::slot_cboRefreshRate_activated(int idx)
{
	QT_THREAD_GUARD()
	switch (idx) {
		case CBO_NO_REFRESH: refreshTimer.stop(); break;
		case CBO_REFRESH_HALF_FPS: refreshTimer.changeInterval(2000); break;
		case CBO_REFRESH_1_FPS: refreshTimer.changeInterval(1000); break;
		case CBO_REFRESH_2_FPS: refreshTimer.changeInterval(500); break;
		case CBO_REFRESH_5_FPS: refreshTimer.changeInterval(200); break;
		case CBO_REFRESH_10_FPS: refreshTimer.changeInterval(100); break;
		default: RDK_ERROR_PRINTF("Error, this refresh rate is not implemented"); break;
	}
}

void ImageViewerGlModule::slot_btnChangeColorPoseSelected_clicked()
{
	QT_THREAD_GUARD()
	QListViewItem* itm = lboxShownPoses->currentItem();
	if (!itm) return;
	vector<string> v = TextUtils::tokenize(itm->text(2).latin1(), ",");
	if (v.size() != 3) {
		RDK_ERROR_PRINTF("Malformed cell value (%s)", itm->text(2).latin1());
	}
	else {
		QColor c = QColorDialog::getColor(QColor(atoi(v[0].c_str()), atoi(v[1].c_str()), atoi(v[2].c_str())), sideWidget);
		if (c.isValid()) {
			itm->setText(2, QString("%1,%2,%3").arg(c.red()).arg(c.green()).arg(c.blue()));
			QPixmap pm(36, 12);
			pm.fill(c);
			itm->setPixmap(1, pm);
		}
	}
}

void ImageViewerGlModule::slot_lboxShownMaps_selectionChanged(QListViewItem* itm)
{
	QT_THREAD_GUARD()
	if (itm && itm->isSelected()) {
		Session* guiSession = RqCommon::getGuiSession(getModuleName());
		string url;
		SESSION_TRY_START(guiSession)
		url = itm->text(0).latin1();
		guiSession->lock(url, HERE);
		RImage* rmi = guiSession->getObjectAsL<RImage>(url);
		lblShownMapInfo->setText(QString("<b>Image information:</b><br/>"
									"Image size: %1 x %2 pixels")
									.arg(rmi->getWidth()).arg(rmi->getHeight()));
		guiSession->unlock(url);
		SESSION_END_CATCH_TERMINATE(guiSession)
		if (url != "") {
			sliderMapAlpha->setValue(atoi(itm->text(1).latin1()));
			sliderMapAlpha->show();
			lblMapAlpha->show();
			btnRemoveShownMapSelected->show();
			lblSelectedMap->setText(url);
		}
	}
	else {
		lblSelectedMap->setText("(no map selected)");
		lblShownMapInfo->setText("");
		sliderMapAlpha->hide();
		lblMapAlpha->hide();
		btnRemoveShownMapSelected->hide();
	}
}

void ImageViewerGlModule::slot_lboxShownPoses_selectionChanged(QListViewItem* itm)
{
	QT_THREAD_GUARD()
	if (itm && itm->isSelected()) {
		Session* guiSession = RqCommon::getGuiSession(getModuleName());
		string url;
		Point2od pose;
		SESSION_TRY_START(guiSession)
		url = itm->text(0).latin1();
		pose = guiSession->getPose(url);
		SESSION_END_CATCH_TERMINATE(guiSession)
		if (url != "") {
			lblSelectedPose->setText(url);
			lblShownPoseInfo->setText(QString("<b>Current pose</b>: %1 %2 %3&deg;").arg(pose.x, 0, 'f', 2)
				.arg(pose.y, 0, 'f', 2).arg(rad2deg(pose.theta), 0, 'f', 2));
			btnChangeColorPoseSelected->show();
			btnRemoveShownPoseSelected->show();
		}
	}
	else {
		lblSelectedPose->setText("(no pose selected)");
		lblShownPoseInfo->setText("");
		btnChangeColorPoseSelected->hide();
		btnRemoveShownPoseSelected->hide();
	}
}

QWidget* ImageViewerGlModule::createSideWidget(QWidget* parent)
{
	QT_THREAD_GUARD(0)
	QTabWidget* tabw = new QTabWidget(parent);
	tabw->setMaximumWidth(300);
	tabw->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
	QFont font = tabw->font();
	font.setPointSize(8);
	tabw->setFont(font);
	
	{
		QWidget* tab = new QWidget(tabw);
		tabw->addTab(tab, "General");
		QVBoxLayout* vl = new QVBoxLayout(tab, 10);
		
		vl->addWidget(new QLabel("<b>How to move, rotate, etc.:</b><br/>"
			"Left-click and drag to move the image<br/>"
			"CTRL + left-click to rotate<br/>"
			"SHIFT + left-click to set pose or draw (see next tabs)", tab));
		
			gboxImageScale = new QGroupBox(1, Qt::Vertical, tab);
				gboxImageScale->setFrameStyle(QFrame::Box | QFrame::Raised);
				gboxImageScale->setTitle(QString(LBL_IMAGE_SCALE).arg(100));
				sliderImageScale = new QSlider(10, 1000, 10, 100, Qt::Horizontal, gboxImageScale);
				sliderImageScale->show();
					QObject::connect(sliderImageScale, SIGNAL(valueChanged(int)), this, SLOT(slot_sliderImageScale_valueChanged(int)));
		vl->addWidget(gboxImageScale);
		
			QGroupBox* gbox = new QGroupBox(2, Qt::Vertical, tab);
				gbox->setMargin(20);
				gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
				gbox->setTitle("Display options");
				
				cboRefreshRate = new QComboBox(gbox);
					cboRefreshRate->insertItem("Never refresh", CBO_NO_REFRESH);
					cboRefreshRate->insertItem("0.5 frame/s",   CBO_REFRESH_HALF_FPS);
					cboRefreshRate->insertItem("1 frame/s",     CBO_REFRESH_1_FPS);
					cboRefreshRate->insertItem("2 frame/s",     CBO_REFRESH_2_FPS);
					cboRefreshRate->insertItem("5 frame/s",     CBO_REFRESH_5_FPS);
					cboRefreshRate->insertItem("10 frame/s",    CBO_REFRESH_10_FPS);
					cboRefreshRate->setCurrentItem(CBO_REFRESH_2_FPS);
					QObject::connect(cboRefreshRate, SIGNAL(activated(int)), this, SLOT(slot_cboRefreshRate_activated(int)));
				
				chkCloneImage = new QCheckBox("Clone image instead of lock it", gbox);
		vl->addWidget(gbox);
		
			gbox = new QGroupBox(3, Qt::Vertical, tab);
				gbox->setMargin(20);
				gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
				gbox->setTitle("Poses");
				
				new QLabel(QString("SHIFT + Left mouse click"), gbox);
				new QLabel(QString("will set this pose:"), gbox);
				lboxEditablePoses = new QListBox(gbox);
		vl->addWidget(gbox);
		
		vl->addStretch(1);
	}
	
	{
		QWidget* tab = new QWidget(tabw);
		tabw->addTab(tab, "Show");
		QVBoxLayout* vl = new QVBoxLayout(tab);
		
			QTabWidget* tabwp = new QTabWidget(tab);
				QWidget* tabp = new QWidget(tabwp);
				tabwp->addTab(tabp, "Poses");
				QVBoxLayout* vll = new QVBoxLayout(tabp);
					QGroupBox* gbox = new QGroupBox(1, Qt::Vertical, tabp);
					gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
					gbox->setTitle("Shown poses");
					lboxShownPoses = new QListView(gbox);
					lboxShownPoses->addColumn("URL");
					lboxShownPoses->addColumn("Color");
					lboxShownPoses->addColumn("Color");
					lboxShownPoses->addColumn("Geometry");
					lboxShownPoses->hideColumn(2);
					lboxShownPoses->hideColumn(3);
					QObject::connect(lboxShownPoses, SIGNAL(selectionChanged(QListViewItem*)),
						this, SLOT(slot_lboxShownPoses_selectionChanged(QListViewItem*)));
				vll->addWidget(gbox);
					gbox = new QGroupBox(4, Qt::Vertical, tabp);
					gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
					gbox->setTitle("Selected pose");		
					lblSelectedPose = new QLabel("...", gbox);		
					btnRemoveShownPoseSelected = new QPushButton("Remove", gbox);
					btnChangeColorPoseSelected = new QPushButton("Change color", gbox);
					QObject::connect(btnChangeColorPoseSelected, SIGNAL(clicked()), this, SLOT(slot_btnChangeColorPoseSelected_clicked()));
					lblShownPoseInfo = new QLabel("", gbox);
				vll->addWidget(gbox);
				slot_lboxShownPoses_selectionChanged(0);
				
				tabp = new QWidget(tabwp);
				tabwp->addTab(tabp, "Maps");
				vll = new QVBoxLayout(tabp);
					gbox = new QGroupBox(1, Qt::Vertical, tabp);
					gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
					gbox->setTitle("Shown maps");
					lboxShownMaps = new QListView(gbox);
					lboxShownMaps->addColumn("URL");
					lboxShownMaps->addColumn("Alpha");
					QObject::connect(lboxShownMaps, SIGNAL(selectionChanged(QListViewItem*)),
						this, SLOT(slot_lboxShownMaps_selectionChanged(QListViewItem*)));
				vll->addWidget(gbox);
					gbox = new QGroupBox(4, Qt::Vertical, tabp);
					gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
					gbox->setTitle("Selected map");
					lblSelectedMap = new QLabel("...", gbox);
					btnRemoveShownMapSelected = new QPushButton("Remove", gbox);
					QWidget* awdg = new QWidget(gbox);
					QHBoxLayout* hl = new QHBoxLayout(awdg);
					hl->addWidget(lblMapAlpha = new QLabel(QString(LBL_MAP_ALPHA).arg(100), awdg));
					hl->addWidget(sliderMapAlpha = new QSlider(0, 100, 10, 100, Qt::Horizontal, awdg));
					QObject::connect(sliderMapAlpha, SIGNAL(valueChanged(int)), this, SLOT(slot_sliderMapAlpha_valueChanged(int)));
					lblShownMapInfo = new QLabel("<b>Click on a map to view its properties</b>", gbox);
				vll->addWidget(gbox);
				slot_lboxShownMaps_selectionChanged(0);
				
				tabp = new QWidget(tabwp);
				tabwp->addTab(tabp, "Other");
				vll = new QVBoxLayout(tabp);
					gbox = new QGroupBox(3, Qt::Vertical, tabp);
					gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
					gbox->setTitle("Properties to show");
						lboxShownProps = new QListView(gbox);
						lboxShownProps->installEventFilter(this);
						lboxShownProps->setAcceptDrops(true);
						lboxShownProps->addColumn("Property name");
				vll->addWidget(gbox);
				
		vl->addWidget(tabwp);
	}
	
	{
		QWidget* tab = new QWidget(tabw);
		tabw->addTab(tab, "Edit");
		QVBoxLayout* vl = new QVBoxLayout(tab, 10);
		
			QLabel* lbl = new QLabel("<b>Edit maps:</b>You can draw on maps holding down SHIFT key and left mouse button, "
				"choose the desired color and pen size below.", tab);
		vl->addWidget(lbl);
		
			QGroupBox* gbox = new QGroupBox(1, Qt::Vertical, tab);
				gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
				gbox->setTitle("Target map");
				cboEditTargetMap = new QComboBox(gbox);
		vl->addWidget(gbox);
		
			gbox = new QGroupBox(1, Qt::Vertical, tab);
				gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
				gbox->setTitle("Pen size");
				cboEditPenSize = new QComboBox(gbox);
				cboEditPenSize->insertItem("Tiny (1x1 pixels)",   CBO_PEN_TINY);
				cboEditPenSize->insertItem("Small (2x2 pixels)",  CBO_PEN_SMALL);
				cboEditPenSize->insertItem("Medium (5x5 pixels)", CBO_PEN_MEDIUM);
				cboEditPenSize->insertItem("Big (10x10 pixels)",  CBO_PEN_BIG);
				cboEditPenSize->setCurrentItem(CBO_PEN_SMALL);
		vl->addWidget(gbox);
						
			gbox = new QGroupBox(1, Qt::Vertical, tab);
				gbox->setFrameStyle(QFrame::Box | QFrame::Raised);
				gbox->setTitle("Pen color");
				lboxEditPenColor = new QListBox(gbox);
				QPixmap pm(12, 12);
				pm.fill(Qt::black);
				lboxEditPenColor->insertItem(new QListBoxPixmap(pm, "Black (obstacles)"),  CBO_COLOR_BLACK);
				pm.fill(Qt::white);
				lboxEditPenColor->insertItem(new QListBoxPixmap(pm, "White (free space)"), CBO_COLOR_WHITE);
				pm.fill(Qt::blue);
				lboxEditPenColor->insertItem(new QListBoxPixmap(pm, "Blue (unknown)"),     CBO_COLOR_BLUE);
				pm.fill(Qt::red);
				lboxEditPenColor->insertItem(new QListBoxPixmap(pm, "Red"),     CBO_COLOR_RED);
				pm.fill(Qt::green);
				lboxEditPenColor->insertItem(new QListBoxPixmap(pm, "Green"),   CBO_COLOR_GREEN);
				pm.fill(Qt::magenta);
				lboxEditPenColor->insertItem(new QListBoxPixmap(pm, "Magenta"), CBO_COLOR_MAGENTA);
				pm.fill(Qt::cyan);
				lboxEditPenColor->insertItem(new QListBoxPixmap(pm, "Cyan"),    CBO_COLOR_CYAN);
				pm.fill(Qt::gray);
				lboxEditPenColor->insertItem(new QListBoxPixmap(pm, "Gray"),    CBO_COLOR_GRAY);
				lboxEditPenColor->setCurrentItem(CBO_COLOR_BLACK);
		vl->addWidget(gbox);
						
		vl->addStretch(1);
	}
	
	updateSidePanelWithConfig();
	
	QObject::connect(&timerSaveToConfig, SIGNAL(timeout()), this, SLOT(slot_timerSaveToConfig_timeout()));
	timerSaveToConfig.start(500);

	return tabw;
}

bool ImageViewerGlModule::eventFilter(QObject *o, QEvent *e)
{
	QT_THREAD_GUARD(false)
	if (o == lboxShownProps) {
		if (e->type() == QEvent::Drop) {
			QDropEvent* event = dynamic_cast<QDropEvent*>(e);
			if (event) {
				QString qsUrl;
				if (QTextDrag::decode(event, qsUrl)) {
					Url u = qsUrl.latin1();
					Session* guiSession = RqCommon::getGuiSession(getModuleName());
					SESSION_TRY_START(guiSession)
					if (guiSession->getPropertyClassName(u) == "RItemOnMapVector") {
						QCheckListItem* itm = new QCheckListItem(lboxShownProps, u, QCheckListItem::CheckBox);
						itm->setOn(true);
					}
					SESSION_END_CATCH_TERMINATE(guiSession)
				}
			}
		}
		else if (e->type() == QEvent::DragEnter) {
			QDragEnterEvent* event = dynamic_cast<QDragEnterEvent*>(e);
			if (event) event->accept(QTextDrag::canDecode(event));
		}
	}
	return false;
}

QWidget* ImageViewerGlModule::createViewerWidget(QWidget* parent)
{
	QT_THREAD_GUARD(0)
	RqGlViewer* glv = new RqGlViewer(parent);
	QObject::connect(glv, SIGNAL(signal_mousePressed(double, double, Qt::ButtonState)),
		this, SLOT(slot_viewerWidget_mousePressed(double, double, Qt::ButtonState)));
	QObject::connect(glv, SIGNAL(signal_mouseMoved(double, double, Qt::ButtonState)),
		this, SLOT(slot_viewerWidget_mouseMoved(double, double, Qt::ButtonState)));
	QObject::connect(glv, SIGNAL(signal_mouseReleased(double, double, Qt::ButtonState)),
		this, SLOT(slot_viewerWidget_mouseReleased(double, double, Qt::ButtonState)));
	QObject::connect(glv, SIGNAL(signal_scaleChanged(double)), this, SLOT(slot_viewerWidget_scaleChanged(double)));
	return glv;
}

void ImageViewerGlModule::slot_viewerWidget_mousePressed(double x, double y, Qt::ButtonState bs)
{
	if ((bs & Qt::LeftButton) && (bs & Qt::ShiftButton)) {
		posing = true;
		posingX = x;
		posingY = y;
	}
}

void ImageViewerGlModule::slot_viewerWidget_mouseMoved(double x, double y, Qt::ButtonState bs)
{
	QT_THREAD_GUARD()
	if (!(bs & Qt::LeftButton)) {
		lblStatus->setText(QString("(%1, %2)").arg((int) (x * 50)).arg((int) -(y * 50)));
	}
	else {
		if (posing) {
			if (lboxEditablePoses->currentItem() != -1) {
				double theta = atan2(y - posingY, x - posingX);
				lblStatus->setText(QString("<span>Setting pose (%1, %2, %3&deg;)</span>").arg(posingX, 0, 'f', 2)
					.arg(posingY, 0, 'f', 2)
					.arg(rad2deg(theta), 0, 'f', 2));
			}
			else {
				lblStatus->setText("Choose a pose to edit, in the 'General' tab");
			}
		}
	}
}

void ImageViewerGlModule::slot_viewerWidget_mouseReleased(double x, double y, Qt::ButtonState /*bs*/)
{
	QT_THREAD_GUARD()
	if (posing) {
		if (lboxEditablePoses->currentItem() != -1) {
			string url = lboxEditablePoses->currentText().latin1();
			double theta = (x == posingX && y == posingY ? DBL_MAX : atan2(y - posingY, x - posingX));
			Session* guiSession = RqCommon::getGuiSession(getModuleName());
			SESSION_TRY_START(guiSession)
			guiSession->setPose(url, Point2od(posingX, posingY, theta));
			SESSION_END_CATCH_TERMINATE(guiSession)
		}
	}
	posing = false;
}

void ImageViewerGlModule::slot_viewerWidget_scaleChanged(double s)
{
	QT_THREAD_GUARD()
	currentScale = (int) (s * 100);
	sliderImageScale->setValue((int) (s * 100));
}

}} // ns
