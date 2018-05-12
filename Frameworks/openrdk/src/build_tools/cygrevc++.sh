#!/bin/bash

LINKLIB=
CMDLINE=

for p in $@
do
	if [ "${p:0:2}" == "-l" ]; then
		LINKLIB="$LINKLIB $p"
	else
		CMDLINE="$CMDLINE $p"
	fi
done

/usr/bin/c++.exe $CMDLINE $LINKLIB
