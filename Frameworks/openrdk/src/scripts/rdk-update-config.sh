#!/bin/bash

if [ $# -ne 1 ]; then
	echo "USAGE: `basename $0` <config-file>"
	exit
fi

cp $1 $1.bak
cat $1.bak | sed -e 's/rdk2r/rdkr/g' > $1
