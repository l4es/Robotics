#!/bin/bash

if [ -z "$RDK_ROOT" ]; then
	echo "ERROR: the environment variable RDK_ROOT is not set."
	echo "Please run '. setenv' or equivalent from OpenRDK source dir,"
	echo "or source it in your .bashrc."
	exit -1
fi

if [ $# -lt 1 ]
then
	echo "Usage: `basename $0` -m <module-name> [-t <template>]"
	echo "       `basename $0` -l"
	echo "       `basename $0` -i"
	echo
	echo "Use '-t' flag to change default template module directory"
	echo "Use '-l' flag to list the template module directories currently installed in your system"
	echo "Use '-i' for interactive mode (you will be asked to enter the required information)"
	echo "Note: <module-name> should be in java-like class notation,"
	echo "with uppercase letters in the beginning of each word:"
	echo "e.g. MyVerySimpleSample"
	echo "Word 'Module' will be automatically added at the end."

	exit -1
fi

templaterootdir=$RDK_ROOT/src/scripts/module-templates
lowercasedtemplate=defaulttemplatemodule
templatedir=${templaterootdir}/${lowercasedtemplate}
module=""

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

while getopts "m:t:li" opt; do
	case $opt in
	l)
		showinstalled
		exit
		;;

	i)
		showinstalled
		echo
		echo -n "Choose a template for the directory of your module (leave empty for the default): "
		read lowercasedtemplate
		if [ "x${lowercasedtemplate}" == "x" ]; then lowercasedtemplate=defaulttemplatemodule; fi
		if [ ! -d $templaterootdir/$lowercasedtemplate ]; then
			if [ -e $templaterootdir/$lowercasedtemplate ]; then
				templatedir=`head -n 1 $templaterootdir/$lowercasedtemplate`
				lowercasedtemplate=`basename $templatedir`
			fi
		else
			templatedir=${templaterootdir}/$lowercasedtemplate
		fi
		echo 
		echo "Choose a name for your module (Java-like upper/lower cases,"
		echo -n " start with uppercase, do not add Module at the end): "
		read mod
		if test "`echo ${mod} | tr A-Z a-z | grep module$`" != ""
		then
			mod=`echo ${mod} | awk '{ print substr($0,1,length($0)-6) }'`
		fi
		module=${mod}Module
		;;

	m)
		mod=${OPTARG}
		if test "`echo ${OPTARG} | tr A-Z a-z | grep module$`" != ""
		then
			mod=`echo ${OPTARG} | awk '{ print substr($0,1,length($0)-6) }'`
		fi
		module=${mod}Module
		;;

	t)
		lowercasedtemplate=`echo $OPTARG | tr A-Z a-z`
		if [ ! -d $templaterootdir/$lowercasedtemplate ]; then
			if [ -e $templaterootdir/$lowercasedtemplate ]; then
				lowercasedtemplate=`head -n 1 $templaterootdir/$lowercasedtemplate`
			fi
		fi
		templatedir=${templaterootdir}/${lowercasedtemplate}
		;;

	*)
		echo "Error on parameters: $opt"
		exit 1
		;;
	esac
done

echo

if test -z $module
then
	echo "ERROR: you must provide at least the module name"
	exit -1
fi

if ! test -d $templatedir
then
	echo "ERROR: your template dir for '$templatedir' ($lowercasedtemplate) does not exists"
	exit -1
fi

test1=`echo $module | grep ^[A-Z]`
if [ "$test1" == "" ]
then
	echo "ERROR: In the <module-name> you provided, the first letter is not uppercase"
	exit -1
fi

lowercased=`echo $module | tr A-Z a-z`
classname=$module
uppercased=`echo $module | tr a-z A-Z`

echo "- module class name: '$classname'"
echo "- module directory: `pwd`/$lowercased"
echo "- template directory: $templatedir"

# Copy the directory
cp -r $templatedir $lowercased
rm -rf $lowercased/.svn
rm -rf $lowercased/CMakeFiles
rm -f $lowercased/DartTestfile.txt
rm -f $lowercased/Makefile
rm -f $lowercased/cmake_install.cmake

# Adjust the template files
sed -e "s/TemplateModule/$classname/g" -e "s/TEMPLATEMODULE/$uppercased/g" $lowercased/templatemodule.h > $lowercased/$lowercased.h
rm $lowercased/templatemodule.h

sed -e "s/TemplateModule/$classname/g" -e "s/templatemodule/$lowercased/g" $lowercased/templatemodule.cpp > $lowercased/$lowercased.cpp
rm $lowercased/templatemodule.cpp

if [ -e $lowercased/templatemodule_names.h ]; then
	mv $lowercased/templatemodule_names.h $lowercased/${lowercased}_names.h
fi

if [ -e $lowercased/CMakeLists.txt ]; then
	sed -e "s/TemplateModule/$classname/g" -e "s/templatemodule/$lowercased/g" -e "s/TEMPLATEMODULE/$uppercased/g" $lowercased/CMakeLists.txt > $lowercased/newCMakeLists.txt
	rm $lowercased/CMakeLists.txt
	mv $lowercased/newCMakeLists.txt $lowercased/CMakeLists.txt
else
	cd $lowercased
	rdk-cmake-ragent-module.sh >/dev/null
	cd ..
fi

echo "- the $classname module directory has been created, have fun!"
echo 
echo "NOTE: feel free to svn add it when you like, but remember to delete the Makefile before svn add"
echo "and issue svn propset svn:ignore Makefile <yourmoduledir> after the svn add and before the svn commit"
echo
