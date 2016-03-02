#!/bin/bash

# A simple bash script to visualize the oldHull, newHull, and mergedHull of one frame.
# Pass Frame number as first parameter. Pass SurfaceNumber as second parameter.

cd build/GnuPlots && gnuplot -e "splot 'OldHull_$1_$2.dat' with lines, 'NewHull_$1_$2.dat' with lines, 'MergeHull_$1_$2.dat' with lines; pause -1"