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

#include "rchecklistitem.h"

std::map<QCheckListItem::ToggleState, StateChanger*> createMap()
{
	std::map<QCheckListItem::ToggleState, StateChanger*> cmap;
	cmap[QCheckListItem::On] = new OnStateChanger();
	cmap[QCheckListItem::NoChange] = new NoChangeStateChanger();
	cmap[QCheckListItem::Off] = new OffStateChanger();
	return cmap;
}

std::map<QCheckListItem::ToggleState, StateChanger*> RCheckListItem::changerMap = createMap();

RCheckListItem::RCheckListItem(
	QListView *parent,
	const QString &text,
	Type tt ):
	QCheckListItem(parent, text, tt)
{
}

void RCheckListItem::paintCell( QPainter *p, const QColorGroup & cg, int column, int width, int align)
{
	////CustomCG.setColor(QColorGroup::Text, "orange");
	////CustomCG.setColor(QColorGroup::ButtonText, "blue");
	//CustomCG.setColor(QColorGroup::Button, "magenta");
	////CustomCG.setColor(QColorGroup::Background, "green");
	////CustomCG.setColor(QColorGroup::Foreground, "red");
	////CustomCG.setColor(QColorGroup::HighlightedText, "yellow");
	QColorGroup ccg(cg);
	changerMap[state()]->setColor(ccg);
	QCheckListItem::paintCell(p, ccg, column, width, align);
	changerMap[state()]->restoreColor(ccg);
}
