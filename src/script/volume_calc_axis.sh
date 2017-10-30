#!/bin/bash

# check that AM2B_ROOT & LEPP_BIN_DIR env variables are set
"${AM2B_ROOT?The AM2B_ROOT environment variable must be set to the root directory of the am2b project!}"
"${LEPP_ROOT?The LEPP_ROOT environment variable must be set to the root directory of the lepp3 project!}"
"${LEPP_BIN_DIR?The LEPP_BIN_DIR environment variable must be set to the directory containing the lepp3 binaries!}"
echo "Using am2b from: $AM2B_ROOT"
WORK_DIR=${PWD}
PCD_CREATION_DIR=$AM2B_ROOT/etc/model/pcd_creation
echo "Storing results in: $WORK_DIR"

export LEPP3_TIMEOUT=30s

object="object"
echo -n -e "Enter the object you want to evaluate\n"
read object
echo -e "Evaluation of $object :\n"
splitting_step=1
echo -n -e "Enter the number of splitting steps (0,1 or 2)\n"
read splitting_step
cd $LEPP_ROOT/config
sed -i "104s/.*/     depth = $splitting_step/" artificial_pcd.toml
if [ "$object" = "box" ]; then
  file=box_vol.wrl
  cd $PCD_CREATION_DIR/lab_scene/
  if [ ! -e "$file" ] ; then
    echo 'objects = (
    {
      url_wrl = "box";
      translation = [0.0, 0.0, 0.0];
      rotation = [0.0, 0.0, 0.0];
      scale = [0.870000, 0.600000, 0.600000];
      velocity = [0.1, 0.1, 0.0];
    });' > box_vol.wrl
  fi;
  cd $WORK_DIR
  range1=80
  range2=180
  DIFF=$(($range2-$range1+1))
  RANDOM=$$
  for i in `seq 20`
  do
      R_x=$(($(($RANDOM%DIFF))+$range1))
      echo $R_x
      scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
      echo $scale_x
      R_y=$(($(($RANDOM%$DIFF))+$range1))
      echo $R_y
      scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
      echo $scale_y
      R_z=$(($(($RANDOM%DIFF))+$range1))
      echo $R_z
      scale_z=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
      echo $scale_z
      cd $LEPP_ROOT/config/
      sed -i "124,127s/.*/#/" artificial_pcd.toml
      sed -i "95s/.*/split_axis = \"smallest\"/" artificial_pcd.toml
      cd $PCD_CREATION_DIR/
      sed -i "3s/.*/-0.5 0 1/" pov.txt
      cd $PCD_CREATION_DIR/lab_scene/
      sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
      cd $PCD_CREATION_DIR/build/
      ./pcd_creator -f "$file" -p --with-floor
      #cd ../../../../../lepp3/build
      cd $LEPP_BIN_DIR
      timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
      original_volume=0.016
      echo "Original volume of $object: $original_volume"
      final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
      echo "Scale factor x: $scale_x"
      echo "Scale factor y: $scale_y"
      echo "Scale factor z: $scale_z"
      echo "Volume after application of scale factors: $final_volume"
      cd $LEPP_ROOT/evaluation/
      fn=$(ls -t | head -n1)
      cd $fn
      IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
      est_smallest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
      echo "Estimated volume of $object (scale factors applied): $est_smallest"
      diff_smallest=$(echo $final_volume\-$est_smallest | bc -l | awk '{printf "%f", $0}')
      if (( $(echo "$diff_smallest < 0" | bc -l) )); then
        ratio_smallest=$(echo $final_volume\/$est_smallest | bc -l | awk '{printf "%f", $0}')
      else
        ratio_smallest=$(echo $est_smallest\/$final_volume | bc -l | awk '{printf "%f", $0}')
      fi;
      echo "The difference between calculated and estimated volume is: $diff_smallest"
      cd $WORK_DIR
      file_smallest=data_smallest.csv
      if [ ! -e "$file_smallest" ]; then
        touch $file_smallest
      fi;
      if [ "$i" == 1 ]; then
        printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Smallest Ratio_Smallest | paste -sd ' ' >> $file_smallest
      fi;
      echo $scale_x $scale_y $scale_z $final_volume $est_smallest $ratio_smallest >> $file_smallest
      cd $LEPP_ROOT/config/
      sed -i "95s/.*/split_axis = \"middle\"/" artificial_pcd.toml
      cd $PCD_CREATION_DIR/build/
      ./pcd_creator -f "$file" -p --with-floor
      cd $LEPP_BIN_DIR
      timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
      cd $LEPP_ROOT/evaluation/
      fn=$(ls -t | head -n1)
      cd $fn
      IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
      est_middle=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
      diff_middle=$(echo $final_volume\-$est_middle | bc -l | awk '{printf "%f", $0}')
      if (( $(echo "$diff_middle < 0" | bc -l) )); then
        ratio_middle=$(echo $final_volume\/$est_middle | bc -l | awk '{printf "%f", $0}')
      else
        ratio_middle=$(echo $est_middle\/$final_volume | bc -l | awk '{printf "%f", $0}')
      fi;
      cd $WORK_DIR
      file_middle=data_middle.csv
      if [ ! -e "$file_middle" ]; then
        touch $file_middle
      fi;
      if [ "$i" == 1 ]; then
        printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Middle Ratio_Middle | paste -sd ' ' >> $file_middle
      fi;
      echo $scale_x $scale_y $scale_z $final_volume $est_middle $ratio_middle >> $file_middle
      cd $LEPP_ROOT/config/
      sed -i "95s/.*/split_axis = \"largest\"/" artificial_pcd.toml
      cd $PCD_CREATION_DIR/build/
      ./pcd_creator -f "$file" -p --with-floor
      cd $LEPP_BIN_DIR
      timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
      cd $LEPP_ROOT/evaluation/
      fn=$(ls -t | head -n1)
      cd $fn
      IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
      est_largest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
      diff_largest=$(echo $final_volume\-$est_largest | bc -l | awk '{printf "%f", $0}')
      if (( $(echo "$diff_largest < 0" | bc -l) )); then
        ratio_largest=$(echo $final_volume\/$est_largest | bc -l | awk '{printf "%f", $0}')
      else
        ratio_largest=$(echo $est_largest\/$final_volume | bc -l | awk '{printf "%f", $0}')
      fi;
      cd $WORK_DIR
      file_largest=data_largest.csv
      if [ ! -e "$file_largest" ]; then
        touch $file_largest
      fi;
      if [ "$i" == 1 ]; then
        printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Largest Ratio_Largest | paste -sd ' ' >> $file_largest
      fi;
      echo $scale_x $scale_y $scale_z $final_volume $est_largest $ratio_largest >> $file_largest
  done
  elif [ "$object" = "beam" ]; then
    file=beam_vol.wrl
    cd $PCD_CREATION_DIR/lab_scene/
    if [ ! -e "$file" ] ; then
      echo 'objects = (
      {
        url_wrl = "beam";
        translation = [0.0, 0.0, 0.0];
        rotation = [1.5708, 0.0, 0.0];
        scale = [0.750000, 0.560000, 0.930000];
        velocity = [0.1, 0.1, 0.0];
      });' > beam_vol.wrl
    fi;
    cd $WORK_DIR
    range1=80
    range2=180
    DIFF=$(($range2-$range1+1))
    RANDOM=$$
    for i in `seq 20`
    do
        R_x=$(($(($RANDOM%DIFF))+$range1))
        echo $R_x
        scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_x
        R_y=$(($(($RANDOM%$DIFF))+$range1))
        echo $R_y
        scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_y
        R_z=$(($(($RANDOM%DIFF))+$range1))
        echo $R_z
        scale_z=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_z
        #cd ../../../am2b/etc/model/pcd_creation/lab_scene
        cd $LEPP_ROOT/config/
        sed -i "124,127s/.*/#/" artificial_pcd.toml
        sed -i "95s/.*/split_axis = \"smallest\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/
        sed -i "3s/.*/-0.5 0 1/" pov.txt
        cd $PCD_CREATION_DIR/lab_scene/
        sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        #cd ../../../../../lepp3/build
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        original_volume=0.0035
        echo "Original volume of $object: $original_volume"
        final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
        echo "Scale factor x: $scale_x"
        echo "Scale factor y: $scale_y"
        echo "Scale factor z: $scale_z"
        echo "Volume after application of scale factors: $final_volume"
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_smallest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of $object (scale factors applied): $est_smallest"
        diff_smallest=$(echo $final_volume\-$est_smallest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_smallest < 0" | bc -l) )); then
          ratio_smallest=$(echo $final_volume\/$est_smallest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_smallest=$(echo $est_smallest\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        echo "The difference between calculated and estimated volume is: $diff_smallest"
        cd $WORK_DIR
        file_smallest=data_smallest.csv
        if [ ! -e "$file_smallest" ]; then
          touch $file_smallest
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Smallest Ratio_Smallest | paste -sd ' ' >> $file_smallest
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_smallest $ratio_smallest >> $file_smallest
        cd $LEPP_ROOT/config/
        sed -i "95s/.*/split_axis = \"middle\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_middle=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        diff_middle=$(echo $final_volume\-$est_middle | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_middle < 0" | bc -l) )); then
          ratio_middle=$(echo $final_volume\/$est_middle | bc -l | awk '{printf "%f", $0}')
        else
          ratio_middle=$(echo $est_middle\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        cd $WORK_DIR
        file_middle=data_middle.csv
        if [ ! -e "$file_middle" ]; then
          touch $file_middle
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Middle Ratio_Middle | paste -sd ' ' >> $file_middle
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_middle $ratio_middle >> $file_middle
        cd $LEPP_ROOT/config/
        sed -i "95s/.*/split_axis = \"largest\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_largest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        diff_largest=$(echo $final_volume\-$est_largest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_largest < 0" | bc -l) )); then
          ratio_largest=$(echo $final_volume\/$est_largest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_largest=$(echo $est_largest\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        cd $WORK_DIR
        file_largest=data_largest.csv
        if [ ! -e "$file_largest" ]; then
          touch $file_largest
        fi;
        if [ "$i" == 1 ]; then
        printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Largest Ratio_Largest | paste -sd ' ' >> $file_largest
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_largest $ratio_largest >> $file_largest
    done
  elif [ "$object" = "cylinder" ]; then
    file=cylinder_vol.wrl
    cd $PCD_CREATION_DIR/lab_scene/
    if [ ! -e "$file" ] ; then
      echo 'objects = (
      {
        url_wrl = "cylinder";
        translation = [0.0, 0.0, 0.0];
        rotation = [0.0, 0.0, 0.0];
        scale = [0.750000, 0.560000, 0.930000];
        velocity = [0.1, 0.1, 0.0];
      });' > cylinder_vol.wrl
    fi;
    cd $WORK_DIR
    range1=90
    range2=190
    DIFF=$(($range2-$range1+1))
    RANDOM=$$
    for i in `seq 20`
    do
        R_x=$(($(($RANDOM%DIFF))+$range1))
        echo $R_x
        scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_x
        R_y=$(($(($RANDOM%$DIFF))+$range1))
        echo $R_y
        scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_y
        R_z=$(($(($RANDOM%DIFF))+$range1))
        echo $R_z
        scale_z=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_z
        #cd ../../../am2b/etc/model/pcd_creation/lab_scene
        cd $PCD_CREATION_DIR/
        sed -i "3s/.*/-0.5 0 1/" pov.txt
        cd $PCD_CREATION_DIR/lab_scene/
        sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        #cd ../../../../../lepp3/build
        cd $LEPP_ROOT/config/
        sed -i "124,127s/.*/#/" artificial_pcd.toml
        sed -i "95s/.*/split_axis = \"smallest\"/" artificial_pcd.toml
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        original_volume=0.000883
        echo "Original volume of $object: $original_volume"
        final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
        echo "Scale factor x: $scale_x"
        echo "Scale factor y: $scale_y"
        echo "Scale factor z: $scale_z"
        echo "Volume after application of scale factors: $final_volume"
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_smallest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of $object (scale factors applied): $est_smallest"
        diff_smallest=$(echo $final_volume\-$est_smallest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_smallest < 0" | bc -l) )); then
          ratio_smallest=$(echo $final_volume\/$est_smallest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_smallest=$(echo $est_smallest\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        echo "The difference between calculated and estimated volume is: $diff_smallest"
        cd $WORK_DIR
        file_smallest=data_smallest.csv
        if [ ! -e "$file_smallest" ]; then
          touch $file_smallest
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Smallest Ratio_Smallest | paste -sd ' ' >> $file_smallest
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_smallest $ratio_smallest >> $file_smallest
        cd $LEPP_ROOT/config/
        sed -i "95s/.*/split_axis = \"middle\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_middle=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        diff_middle=$(echo $final_volume\-$est_middle | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_middle < 0" | bc -l) )); then
          ratio_middle=$(echo $final_volume\/$est_middle | bc -l | awk '{printf "%f", $0}')
        else
          ratio_middle=$(echo $est_middle\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        cd $WORK_DIR
        file_middle=data_middle.csv
        if [ ! -e "$file_middle" ]; then
          touch $file_middle
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Middle Ratio_Middle | paste -sd ' ' >> $file_middle
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_middle $ratio_middle >> $file_middle
        cd $LEPP_ROOT/config/
        sed -i "95s/.*/split_axis = \"largest\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_largest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        diff_largest=$(echo $final_volume\-$est_largest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_largest < 0" | bc -l) )); then
          ratio_largest=$(echo $final_volume\/$est_largest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_largest=$(echo $est_largest\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        cd $WORK_DIR
        file_largest=data_largest.csv
        if [ ! -e "$file_largest" ]; then
          touch $file_largest
        fi;
        if [ "$i" == 1 ]; then
        printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Largest Ratio_Largest | paste -sd ' ' >> $file_largest
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_largest $ratio_largest >> $file_largest
    done
  elif [ "$object" = "triangle" ]; then
    file=triangle_vol.wrl
    cd $PCD_CREATION_DIR/lab_scene/
    if [ ! -e "$file" ] ; then
      echo 'objects = (
      {
        url_wrl = "triangle";
        translation = [0.0, 0.5, 0.0];
        rotation = [ 0.0, -1.5708, 1.5708 ];
        scale = [0.750000, 0.560000, 0.930000];
        velocity = [0.1, 0.1, 0.0];
      });' > triangle_vol.wrl
    fi;
    cd $WORK_DIR
    range1=90
    range2=190
    DIFF=$(($range2-$range1+1))
    RANDOM=$$
    for i in `seq 20`
    do
        R_x=$(($(($RANDOM%DIFF))+$range1))
        echo $R_x
        scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_x
        R_y=$(($(($RANDOM%$DIFF))+$range1))
        echo $R_y
        scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_y
        R_z=$(($(($RANDOM%DIFF))+$range1))
        echo $R_z
        scale_z=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_z
        #cd ../../../am2b/etc/model/pcd_creation/lab_scene
        cd $PCD_CREATION_DIR/
        sed -i "3s/.*/-0.5 0 1/" pov.txt
        cd $PCD_CREATION_DIR/lab_scene/
        sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        #cd ../../../../../lepp3/build
        cd $LEPP_ROOT/config/
        sed -i "124,127s/.*/#/" artificial_pcd.toml
        sed -i "95s/.*/split_axis = \"smallest\"/" artificial_pcd.toml
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        original_volume=0.000826
        echo "Original volume of $object: $original_volume"
        final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
        echo "Scale factor x: $scale_x"
        echo "Scale factor y: $scale_y"
        echo "Scale factor z: $scale_z"
        echo "Volume after application of scale factors: $final_volume"
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_smallest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of $object (scale factors applied): $est_smallest"
        diff_smallest=$(echo $final_volume\-$est_smallest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_smallest < 0" | bc -l) )); then
          ratio_smallest=$(echo $final_volume\/$est_smallest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_smallest=$(echo $est_smallest\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        echo "The difference between calculated and estimated volume is: $diff_smallest"
        cd $WORK_DIR
        file_smallest=data_smallest.csv
        if [ ! -e "$file_smallest" ]; then
          touch $file_smallest
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Smallest Ratio_Smallest | paste -sd ' ' >> $file_smallest
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_smallest $ratio_smallest >> $file_smallest
        cd $LEPP_ROOT/config/
        sed -i "95s/.*/split_axis = \"middle\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_middle=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        diff_middle=$(echo $final_volume\-$est_middle | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_middle < 0" | bc -l) )); then
          ratio_middle=$(echo $final_volume\/$est_middle | bc -l | awk '{printf "%f", $0}')
        else
          ratio_middle=$(echo $est_middle\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        cd $WORK_DIR
        file_middle=data_middle.csv
        if [ ! -e "$file_middle" ]; then
          touch $file_middle
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Middle Ratio_Middle | paste -sd ' ' >> $file_middle
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_middle $ratio_middle >> $file_middle
        cd $LEPP_ROOT/config/
        sed -i "95s/.*/split_axis = \"largest\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        est_largest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        diff_largest=$(echo $final_volume\-$est_largest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_largest < 0" | bc -l) )); then
          ratio_largest=$(echo $final_volume\/$est_largest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_largest=$(echo $est_largest\/$final_volume | bc -l | awk '{printf "%f", $0}')
        fi;
        cd $WORK_DIR
        file_largest=data_largest.csv
        if [ ! -e "$file_largest" ]; then
          touch $file_largest
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Largest Ratio_Largest | paste -sd ' ' >> $file_largest
        fi;
        echo $scale_x $scale_y $scale_z $final_volume $est_largest $ratio_largest >> $file_largest
    done
  elif [ "$object" = "sequence" ]; then
    file=sequence_vol.wrl
    cd $PCD_CREATION_DIR/lab_scene/
    if [ ! -e "$file" ] ; then
      echo 'objects = (
      {
        url_wrl = "box";
        translation = [0.0, -0.3, 0.0];
        rotation = [0.0, 0.0, 0.0];
        scale = [0.5, 0.5, 0.5];
        velocity = [0.1, 0.1, 0.0];
      },
      {
        url_wrl = "beam";
        translation = [0.0, 0.3, 0.0];
        rotation = [1.5708, 0.0, 0.0];
        scale = [0.5, 0.5, 0.5];
        velocity = [0.1, 0.1, 0.0];
      });' > sequence_vol.wrl
    fi;
    cd $WORK_DIR
    range1=80
    range2=180
    DIFF=$(($range2-$range1+1))
    RANDOM=$$
    for i in `seq 20`
    do
        R_x=$(($(($RANDOM%DIFF))+$range1))
        echo $R_x
        scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_x
        R_y=$(($(($RANDOM%$DIFF))+$range1))
        echo $R_y
        scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_y
        R_z=$(($(($RANDOM%DIFF))+$range1))
        echo $R_z
        scale_z=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
        echo $scale_z
        #cd ../../../am2b/etc/model/pcd_creation/lab_scene
        cd $PCD_CREATION_DIR/
        sed -i "3s/.*/-0.5 0 1/" pov.txt
        cd $PCD_CREATION_DIR/lab_scene/
        sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
        sed -i "13s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        #cd ../../../../../lepp3/build
        cd $LEPP_ROOT/config/
        sed -i "124,127s/.*/#/" artificial_pcd.toml
        sed -i "95s/.*/split_axis = \"smallest\"/" artificial_pcd.toml
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola ../config/artificial_pcd.toml
        original_volume_box=0.016
        original_volume_beam=0.0035
        echo "Original volume of box: $original_volume_box"
        echo "Original volume of beam: $original_volume_beam"
        final_volume_box=$(echo $original_volume_box\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
        final_volume_beam=$(echo $original_volume_beam\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
        echo "Scale factor x: $scale_x"
        echo "Scale factor y: $scale_y"
        echo "Scale factor z: $scale_z"
        echo "Box volume after application of scale factors: $final_volume_box"
        echo "Beam volume after application of scale factors: $final_volume_beam"
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        beam_est_smallest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of beam (scale factors applied): $beam_est_smallest"
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -2 eval.csv)
        box_est_smallest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of box (scale factors applied): $box_est_smallest"
        diff_beam_smallest=$(echo $final_volume_beam\-$beam_est_smallest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_beam_smallest < 0" | bc -l) )); then
          ratio_beam_smallest=$(echo $final_volume_beam\/$beam_est_smallest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_beam_smallest=$(echo $beam_est_smallest\/$final_volume_beam | bc -l | awk '{printf "%f", $0}')
        fi;
        echo "The difference between calculated and estimated beam volume is: $diff_beam_smallest"
        diff_box_smallest=$(echo $final_volume_box\-$box_est_smallest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_box_smallest < 0" | bc -l) )); then
          ratio_box_smallest=$(echo $final_volume_box\/$box_est_smallest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_box_smallest=$(echo $box_est_smallest\/$final_volume_box | bc -l | awk '{printf "%f", $0}')
        fi;
        echo "The difference between calculated and estimated box volume is: $diff_box_smallest"
        cd $WORK_DIR
        file_smallest=data_smallest.csv
        if [ ! -e "$file_smallest" ]; then
          touch $file_smallest
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc_Beam Beam_Est_Smallest Beam_Ratio_Smallest Calc_Box Box_Est_Smallest Box_Ratio_Smallest | paste -sd ' ' >> $file_smallest
        fi;
        echo $scale_x $scale_y $scale_z $final_volume_beam $beam_est_smallest $ratio_beam_smallest $final_volume_box $box_est_smallest $ratio_box_smallest >> $file_smallest
        cd $LEPP_ROOT/config/
        sed -i "95s/.*/split_axis = \"middle\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola  ../config/artificial_pcd.toml
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        beam_est_middle=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of beam (scale factors applied): $beam_est_middle"
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -2 eval.csv)
        box_est_middle=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of box (scale factors applied): $box_est_middle"
        diff_beam_middle=$(echo $final_volume_beam\-$beam_est_middle | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_beam_middle < 0" | bc -l) )); then
          ratio_beam_middle=$(echo $final_volume_beam\/$beam_est_middle | bc -l | awk '{printf "%f", $0}')
        else
          ratio_beam_middle=$(echo $beam_est_middle\/$final_volume_beam | bc -l | awk '{printf "%f", $0}')
        fi;
        echo "The difference between calculated and estimated beam volume is: $diff_beam_middle"
        diff_box_middle=$(echo $final_volume_box\-$box_est_middle | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_box_middle < 0" | bc -l) )); then
          ratio_box_middle=$(echo $final_volume_box\/$box_est_middle | bc -l | awk '{printf "%f", $0}')
        else
          ratio_box_middle=$(echo $box_est_middle\/$final_volume_box | bc -l | awk '{printf "%f", $0}')
        fi;
          echo "The difference between calculated and estimated box volume is: $diff_box_middle"
        cd $WORK_DIR
        file_middle=data_middle.csv
        if [ ! -e "$file_middle" ]; then
          touch $file_middle
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc_Beam Beam_Est_Middle Beam_Ratio_Middle Calc_Box Box_Est_Middle Box_Ratio_Middle | paste -sd ' ' >> $file_middle
        fi;
        echo $scale_x $scale_y $scale_z $final_volume_beam $beam_est_middle $ratio_beam_middle $final_volume_box $box_est_middle $ratio_box_middle >> $file_middle
        cd $LEPP_ROOT/config/
        sed -i "95s/.*/split_axis = \"largest\"/" artificial_pcd.toml
        cd $PCD_CREATION_DIR/build/
        ./pcd_creator -f "$file" -p --with-floor
        cd $LEPP_ROOT/build/
        timeout $LEPP3_TIMEOUT ./lola  ../config/artificial_pcd.toml
        cd $LEPP_ROOT/evaluation/
        fn=$(ls -t | head -n1)
        cd $fn
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
        beam_est_largest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of beam (scale factors applied): $beam_est_largest"
        IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -2 eval.csv)
        box_est_largest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
        echo "Estimated volume of box (scale factors applied): $box_est_largest"
        diff_beam_largest=$(echo $final_volume_beam\-$beam_est_largest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_beam_largest < 0" | bc -l) )); then
          ratio_beam_largest=$(echo $final_volume_beam\/$beam_est_largest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_beam_largest=$(echo $beam_est_largest\/$final_volume_beam | bc -l | awk '{printf "%f", $0}')
        fi;
        echo "The difference between calculated and estimated beam volume is: $diff_beam_largest"
        diff_box_largest=$(echo $final_volume_box\-$box_est_largest | bc -l | awk '{printf "%f", $0}')
        if (( $(echo "$diff_box_largest < 0" | bc -l) )); then
          ratio_box_largest=$(echo $final_volume_box\/$box_est_largest | bc -l | awk '{printf "%f", $0}')
        else
          ratio_box_largest=$(echo $box_est_largest\/$final_volume_box | bc -l | awk '{printf "%f", $0}')
        fi;
        echo "The difference between calculated and estimated box volume is: $diff_box_largest"
        cd $WORK_DIR
        file_largest=data_largest.csv
        if [ ! -e "$file_largest" ]; then
          touch $file_largest
        fi;
        if [ "$i" == 1 ]; then
          printf '%s\n' Scale_x Scale_y Scale_z Calc_Beam Beam_Est_Largest Beam_Ratio_Largest Calc_Box Box_Est_Largest Box_Ratio_Largest | paste -sd ' ' >> $file_largest
        fi;
        echo $scale_x $scale_y $scale_z $final_volume_beam $beam_est_largest $ratio_beam_largest $final_volume_box $box_est_largest $ratio_box_largest >> $file_largest
    done
else
  echo "Error. Object is not defined!!!"
  exit 1
fi;
