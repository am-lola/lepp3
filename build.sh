#!/bin/bash

# A simple script that builds the project with only default cmake options set.

dir="build"
dirPlot="GnuPlots"
echo "Building the project..."
echo "Destination directory $dir"

rm -r $dir && mkdir $dir && cd $dir && mkdir $dirPlot && cmake ../src && make -j4
