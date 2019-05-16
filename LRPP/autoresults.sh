#!/bin/bash
# Written by: Florence Tsang
# Creation date: Sept 12 2018
# script for automatically generating sim data, written by Florence

ENV="../environments/maze.dat"
RESDIR="../results/LRPP1_0/maze_Ah/"
START="20 20"
GOAL="20 1"

for tasks in {10..100..10}
do
	echo "T = $tasks"
	for i in {1..10}
	do
		RESFILE="$RESDIR${tasks}_$i.dat"
		echo $RESFILE
		python main.py -e $ENV -s "$START" -g "$GOAL" -t $tasks -o $RESFILE
	done
done

for tasks in {200..500..100}
do
	echo "T = $tasks"
	for i in {1..10}
	do
		RESFILE="$RESDIR${tasks}_$i.dat"
		echo $RESFILE
		python main.py -e $ENV -s "$START" -g "$GOAL" -t $tasks -o $RESFILE
	done
done

