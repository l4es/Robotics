#!/bin/bash

if [ $# -lt 4 ]; then
	echo "OpenRDK offline learning tool (OpenRDK small utilities)"
	echo "USAGE: $(basename $0) <config-file> <epochs> <iterations-for-epoch> <learner-app> [<learner-params>]"
	echo "NOTES:"
	echo "The <learner-app> is assumed to have the following signature:"
	echo "<learner-app> <input-dir> <output-dir> <iterations-for-epoch> [<learner-params>]"
	echo "In the input directory it will find the <iterations-for-epoch> results of the last epoch and"
	echo "generate <iterations-for-epoch> parameter sets for the next epoch, in the output directory."
	echo "In the very first epoch, the learner will receive the same (empty) directory as input and"
	echo "as output, and is requested to generate the first <iterations-for-epoch> parameter sets."
	exit
fi

configfile=$1
epochs=$2
iterations=$3
learner=$4
shift 4
workdir="rdk-learn-results"
rolloutResults="rdk-rollout-results.txt"

seqcmd=seq
if [ ! -e "$(which seq)" ]; then
	if [ -e $(which gseq) ]; then
		# MacOS with DarwinPorts and coreutils installed
		seqcmd=gseq
	else
		echo "You do not have a seq command in your system (if you are on a MacOS, try installing coreutils)"
		exit
	fi
fi

startEpoch=1
startIteration=1
resuming=0

if [ -e $workdir ]; then
	resuming=1
	for e in $($seqcmd -w 1 $epochs); do
		if [ -e $workdir/epoch-$e ]; then
			startEpoch=$e
		else
			break
		fi
	done
	startIteration=0
	for i in $($seqcmd -w 1 $iterations); do
		if [ -e $workdir/epoch-$startEpoch/iteration-$i.txt ]; then
			n=$(grep "\[RESULTS\]" $workdir/epoch-$startEpoch/iteration-$i.txt | wc -l)
			if [ $n -eq 0 ]; then
				startIteration=$i
				break
			fi
		else
			# the files are fewer than expected
			startIteration=1
			break
		fi
	done
	if [ $startIteration -eq 0 ]; then
		ndig=${#startEpoch}
		while [ "${startEpoch:0:1}" == "0" ]; do startEpoch=${startEpoch:1}; done
		startEpoch=$(($startEpoch+1))
		while [ ${#startEpoch} -lt $ndig ]; do startEpoch=0$startEpoch; done
		startIteration=1
	fi
	if [ $startEpoch -gt $epochs ]; then
		echo "Directory $workdir is present and the learning process is finished, please remove or rename the directory before starting another learning process."
		exit
	fi
	echo "Directory $workdir is present: the learning process has been interrupted at epoch $startEpoch of $epochs, iteration $startIteration of $iterations, we will start from there."
	read -p "Do you want to resume a previous learning process (y/N)? " r
	if [ "$r" != "y" ]; then 
		echo "Please remove or rename the $workdir, before starting a new learning process"
		exit
	fi  
fi
if [ $resuming -eq 0 ]; then
	mkdir $workdir
fi

for e in `$seqcmd -w $startEpoch $epochs`; do
	if [ -e $workdir/epoch-$e ]; then
		echo "Warning: directory $workdir/epoch-$e already exists"
	else
		mkdir $workdir/epoch-$e
	fi

	geniterations=0
	ifiles=$(ls -1 $workdir/epoch-$e | wc -l)
	if [ $ifiles -eq 0 ]; then
		geniterations=1
	elif [ $ifiles -lt $iterations ]; then
		read -p "The iteration files in the directory $workdir/epoch-$e are fewer than expected, do you want to regenerate them (y/N)? " r
		if [ "$r" != "y" ]; then
			echo "Please remove the iteration files from the epoch directory"
		fi
		geniterations=1
	fi
	if [ $geniterations -eq 1 ]; then
		if [ $e -eq 1 ]; then
			# create the first epoch
			echo "FIRST EPOCH"
			echo "  Generating initial parameter sets..."
			$learner $workdir/epoch-$e $workdir/epoch-$e $iterations $*
		else
			# generate the next epoch with results from the previous
			echo "EPOCH $e"
			echo "  Learning and generating next parameter sets..."
			ee=$e
			while [ "${ee:0:1}" == "0" ]; do ee=${ee:1}; done
			prevEpoch=$(($ee-1))
			while [ ${#prevEpoch} -lt ${#e} ]; do prevEpoch=0$prevEpoch; done
			$learner $workdir/epoch-$prevEpoch $workdir/epoch-$e $iterations $*
		fi
		if [ $? -ne 0 ]; then
			echo "Something wrong with learner call ($learner)."
			exit -1
		fi
	else
		echo "EPOCH $e"
		echo "  Using previously made iteration files"
	fi

	for i in `$seqcmd -w $startIteration $iterations`; do
		echo "  ITERATION $i"
		echo "    Generating configuration file..."
		rdk-modify-config-properties -in $configfile -m $workdir/epoch-$e/iteration-$i.txt -out current.config
		if [ $? -ne 0 ]; then
			echo "Something wrong with configuration generation."
			exit -1
		fi
		echo "    Configuration generated, now starting RAgent..."
		ragent2 -c current.config
		# echo "32903" > $rolloutResults
		rm current.config
		echo "    RAgent finished, collecting results..."
		echo -e "\nRESULTS:"
		cat $rolloutResults
		echo ""
		sleep 1s
		cp $workdir/epoch-$e/iteration-$i.txt current-results.txt
		echo -e "[RESULTS]" >> current-results.txt
		cat current-results.txt $rolloutResults > $workdir/epoch-$e/iteration-$i.txt
		rm current-results.txt $rolloutResults
	done
	startIteration=1
done

