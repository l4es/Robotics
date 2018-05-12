#!/bin/bash
VERSION=2.2
#===============================================================================
#
#          FILE:  rdk-upload-it.sh
# 
#         USAGE:  ./rdk-upload-it.sh 
# 
#   DESCRIPTION:  Upload compiled code to a remote pc (or a robot :-) )
# 
#       OPTIONS:  ---
#  REQUIREMENTS:  ---
#          BUGS:  ---
#         NOTES:  ---
#        AUTHOR:  Luca Marchetti (), ximilian@gmail.com
#      COAUTHOR:  Gabriel Ulici (NIL), ulicigabriel@gmail.com
#       COMPANY:  OpenRDK
#       VERSION:  $VERSION
#       CREATED:  11/18/2009 05:21:42 PM CET
#      MODIFIED:  09/05/2012 05:25:42 PM CET
#      REVISION:  ---
#===============================================================================
pad=20

root_dir="$RDK_BUILD_DIR"
source_dir="$root_dir"
dest_dir=/home/`whoami`/openrdk
ip_address=""
config_dir=""
dry_run=0
bin=0
fast=0
user=root

h="h"
d="d"
s="s"
c="c"
t="t"
u="u"
f="f"
b="b"

function usage ()
{
	printf \
	"Usage:
   $(basename $0) -${h} <ip-address> [-${d} <dest-dir>] [-${s} <source-dir>] [-${c} <conf-dir>] [-${u} <username>] [-${f}] [-${b}] [-${t}]
\t-%-${pad}s -> %s
\t-%-${pad}s -> %s
\t-%-${pad}s -> %s
\t-%-${pad}s -> %s
\t-%-${pad}s -> %s
\t-%-${pad}s -> %s
\t-%-${pad}s -> %s
\t-%-${pad}s -> %s\n" \
"${h} <ip-address>" "remote IP address" \
"${d} <dest-dir>" "remote destination basedir" \
"${s} <source-dir>" "local source dir (default ${source_dir})" \
"${c} <conf-dir>" "local config dir" \
"${t} " "perform a dry run test" \
"${u} <username>" "remote username (for scp; default: ${user})" \
"${f}" "fast-mode: update config files" \
"${b}" "binary-mode: update binary files"
}    # ----------  end of function usage  ----------

function exec_command()
{
	if [ $# -gt 0 ]
	then
		if [ $dry_run -eq 1 ]
		then
			echo -e "\n$*"
		else
			eval $*
			echo "done."
		fi
	fi
}    # ----------  end of function exec_command  ----------


function header ()
{
	echo "$(basename $0) -- v$VERSION"
	echo "A remote host uploader utility for OpenRDK"
}    # ----------  end of function header  ----------

function main ()
{
	header
	# searching for root directory
	if test -d "$root_dir";
	then
		echo "Found root_dir in [$root_dir]";
	else
		echo "Error: cannot find root_dir in [$root_dir]";
		exit -1
	fi

	# parsing command line
	while getopts ":${s}:${d}:${h}:${c}:${u}:${f}${b}${t}" opt; do
		case $opt in
			${s})
			source_dir=${OPTARG}
			;;
			${d})
			dest_dir=${OPTARG}
			;;
			${c})
			config_dir=${OPTARG}
			;;
			${h})
			ip_address=${OPTARG}
			;;
			${u})
			user=${OPTARG}
			;;
			${f})
			fast=1
			echo "Skipping binary files"
			;;
			${b})
			bin=1
			echo "Skipping config files"
			;;
			${t})
			dry_run=1
			echo "Performing dry-run test"
			;;
			*)
			usage
			;;
		esac
	done

	if test "x$ip_address" == "x"
	then
		echo "Error ip address not specified"
		usage
		exit -1
	fi

	host=$user@${ip_address}

	# should I exclude config files?
	if [ $bin -eq 0 ]
	then
		conffiles=`find "${config_dir}" -type f | grep -v .svn| sed -e "s,[^.],\'&," -e "s,\$,\',"`
	fi

	# should I exclude binary files?
	if [ $fast -eq 0 ]
	then
		binfiles=`find "${source_dir}/bin/" -type f | sed -e "s,[^.],\'&," -e "s,\$,\',"`
		binfiles=${binfiles}" "`find "${source_dir}/binext/" -type f | sed -e "s,[^.],\'&," -e "s,\$,\',"`

		libfiles=`find "${source_dir}/lib/" -type f | sed -e "s,[^.],\'&," -e "s,\$,\',"`
		libfiles=${libfiles}" "`find "${source_dir}/libext/" -type f | sed -e "s,[^.],\'&," -e "s,\$,\',"`
	fi

	config_dir=$(echo $config_dir|sed 's@\(.*\)/@\1@')

	# copy the files
	echo -n "Uploading files to host '${ip_address}' using '${user}' user... "
	cmd="
	tar --checkpoint=.100 -P \
	--transform='s,${config_dir}/\(.*\),config/\1,' \
	--transform='s,\"${source_dir}\",,' \
	--transform='s,.*binext/\(.*\),bin/\1,' \
	--transform='s,.*bin/\(.*\),bin/\1,' \
	--transform='s,.*libext/\(.*\),lib/\1,' \
	--transform='s,.*lib/\(.*\),lib/\1,' \
	--transform='s,^,$dest_dir/,' \
	--exclude=.svn \
	-cf - $binfiles $libfiles $conffiles --totals 
	|
	ssh $host \"
	tar -mC / -xf -
	&&
	echo -e 'export PATH=\\\`pwd\\\`/bin:\\\$PATH\nexport LD_LIBRARY_PATH=\\\`pwd\\\`/lib:\\\$LD_LIBRARY_PATH' > '$dest_dir'/setenv
	&&
	chmod +x '$dest_dir'/setenv
	\""
	exec_command "$cmd"
	echo -e "\nUpload complete."
}    # ----------  end of function main  ----------

main $*

