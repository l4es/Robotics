#!/bin/bash

if [ "x$RDK_ROOT" == "x" ]; then
	echo "The environment variable RDK_ROOT is not set, did you run . setenv ?"
	exit -1
fi

if [ $# -lt 1 ]; then
	echo "USAGE: `basename $0` -i <directory>"
	echo "       `basename $0` -u <template-name>"
	echo "       `basename $0` -l"
	echo
	echo "Install a new template directory using the option -i,"
	echo "uninstall an installed template directory using the option -u,"
	echo "list available template directories using the option -l"
	exit -1
fi

templaterootdir=$RDK_ROOT/src/scripts/module-templates
lowercasedtemplate=defaulttemplatemodule

function showinstalled
{
	echo ""
	echo "These are the module directory templates currently installed on your OpenRDK system:"
	for f in `ls ${templaterootdir}`; do
		if [ -d ${templaterootdir}/$f ]; then
			echo -n "   $f  (built-in)"
		else
			echo -n "   $f  (installed)"
		fi
		if [ "$f" == "$lowercasedtemplate" ]; then echo " (default)"; else echo ""; fi
	done
}


if [ $1 == '-i' ]; then
	if [ "x$2" == "x" ]; then
		echo "Please provide the directory of the template to install"
		exit -1
	fi
	if [ ! -d $2 ]; then
		echo "Directory $2 does not exist (or it is not a directory)"
		exit -1
	fi
	echo `pwd`/$2 > $templaterootdir/`basename $2`
	echo "Template $2 successfully installed."
elif [ "$1" == "-u" ]; then
	if [ "x$2" == "x" ]; then
		echo "Please provide the name of the template to remove (run `basename $0` -l for a list)"
		exit -1
	fi
	if [ ! -e $templaterootdir/$2 ]; then
		echo "The template $2 does not exist (or it is not installed)"
		exit -1
	fi
	if [ ! -f $templaterootdir/$2 ]; then
		echo "The template $2 is a built-in template, you cannot remove it"
		exit -1
	fi
	rm $templaterootdir/$2
	echo "Template $2 successfully uninstalled."
	exit 0
elif [ "$1" == "-l" ]; then
	showinstalled
	exit 0
else
	echo "Unknown option $1"
	exit -1
fi
