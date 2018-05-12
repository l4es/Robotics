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

#ifndef RDK2_RCHECKLISTITEM
#define RDK2_RCHECKLISTITEM

#include <qlistview.h>
#include <qstring.h>
#include <qpalette.h>
#include <qcolor.h>

#include <map>

class StateChanger
{
	public:
		StateChanger(const QColorGroup::ColorRole& _Role, const QString& _Color):
			cr(_Role),
			color(_Color)
		{}
		
		void setColor(QColorGroup& cg)
		{
			previous = cg.color(cr);
			cg.setColor(cr, color);
		}
		void restoreColor(QColorGroup& cg)
		{
			cg.setColor(cr, previous);
		}

		QColorGroup::ColorRole cr;
		QString color;
		QColor previous;
};

class OnStateChanger: public StateChanger
{
	public:
		OnStateChanger():
			StateChanger(QColorGroup::Text, "red")
	{}
};

class NoChangeStateChanger: public StateChanger
{
	public:
		NoChangeStateChanger():
			StateChanger(QColorGroup::Button, "white")
	{}
};

class OffStateChanger: public StateChanger
{
	public:
		OffStateChanger():
			StateChanger(QColorGroup::Button, "white")
	{}
};

/**
 * This file represents a derivation of QCheckListItem.
 * We need it to overload paintCell, for costum behaviour.
 * Mainly, it changes color of checkboxes for better
 * visual feedback.
 */
class RCheckListItem: public QCheckListItem
{
	static std::map<QCheckListItem::ToggleState, StateChanger*> changerMap;

	public:
		RCheckListItem( QListView *parent, const QString &text, Type tt );

	protected:
		virtual void paintCell( QPainter *p, const QColorGroup & cg, int column, int width, int align);
};

#endif
