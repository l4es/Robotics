#include "color.h"

namespace RDK2 { namespace RGraphics
{
	namespace Meta
	{
		ColorToNameReferenceTable& getColorToNameReferenceTable()
		{
			static ColorToNameReferenceTable c;
			return c;
		}

		NameToColorReferenceTable& getNameToColorReferenceTable()
		{
			static NameToColorReferenceTable c;
			return c;
		}

		ColorToNameReference::ColorToNameReference(RgbColor c, std::string n)
		{
			ColorToNameReferenceTable& cRef = getColorToNameReferenceTable();
			cRef[c] = n;
		}

		NameToColorReference::NameToColorReference(std::string n, RgbColor c)
		{
			NameToColorReferenceTable& cRef = getNameToColorReferenceTable();
			cRef[n] = c;
		}
	}

	std::string getColorName(RgbColor color)
	{
		Meta::ColorToNameReferenceTable& cRef = Meta::getColorToNameReferenceTable();
		Meta::ColorToNameReferenceTable::const_iterator it=cRef.find(color);
		if (it!=cRef.end())
		{
			return it->second;
		}
		return "nocolor";
	}

	RgbColor getColorByName(const std::string& name)
	{
		Meta::NameToColorReferenceTable& cRef = Meta::getNameToColorReferenceTable();
		Meta::NameToColorReferenceTable::const_iterator it=cRef.find(name);
		if (it!=cRef.end())
		{
			return it->second;
		}
		return RGB_BLACK;
	}


}}
