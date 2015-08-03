#!/bin/bash

# A simple script that builds the project with only default cmake options set.

dir="build"
echo "Building the project..."
echo "Destination directory $dir"

mkdir $dir && cd $dir && cmake ../ && make -j4
