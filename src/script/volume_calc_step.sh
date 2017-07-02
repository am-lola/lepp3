#!/bin/bash
object="object"
echo -n -e "Enter the object you want to evaluate\n"
read object
echo -e "Evaluation of $object :\n"
split_axis=smallest
echo -n -e "Enter the split axis (smallest, middle, largest)\n"
read split_axis
cd ~/lepp3/
sed -i "77s/.*/  split_axis = \"$split_axis\"/" artificial_pcd.toml
if [ "$object" = "box" ]; then
file=box_vol.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
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
cd ~
range1=50
range2=150
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
    cd ~/lepp3/
    sed -i "106,109s/.*/#/" artificial_pcd.toml
    sed -i "86s/.*/     depth = 0/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 3/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    original_volume=0.016
    echo "Original volume of $object: $original_volume"
    final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Volume after application of scale factors: $final_volume"
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_0_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_0_axis"
    diff_0_axis=$(echo $final_volume\-$est_0_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_0_axis < 0" | bc -l) )); then
    ratio_0_axis=$(echo $final_volume\/$est_0_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_0_axis=$(echo $est_0_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_0_axis"
    cd ~
    file_0_axis=data_0_axis.csv
    if [ ! -e "$file_0_axis" ]; then
    touch $file_0_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_0_Axis Ratio_0_Axis | paste -sd ' ' >> $file_0_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_0_axis $ratio_0_axis >> $file_0_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 1/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_1_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_1_axis"
    diff_1_axis=$(echo $final_volume\-$est_1_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_1_axis < 0" | bc -l) )); then
    ratio_1_axis=$(echo $final_volume\/$est_1_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_1_axis=$(echo $est_1_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_1_axis"
    cd ~
    file_1_axis=data_1_axis.csv
    if [ ! -e "$file_1_axis" ]; then
    touch $file_1_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_1_Axis Ratio_1_Axis | paste -sd ' ' >> $file_1_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_1_axis $ratio_1_axis >> $file_1_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 2/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_2_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_2_axis"
    diff_2_axis=$(echo $final_volume\-$est_2_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_2_axis < 0" | bc -l) )); then
    ratio_2_axis=$(echo $final_volume\/$est_2_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_2_axis=$(echo $est_2_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_2_axis"
    cd ~
    file_2_axis=data_2_axis.csv
    if [ ! -e "$file_2_axis" ]; then
    touch $file_2_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_2_Axis Ratio_2_Axis | paste -sd ' ' >> $file_2_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_2_axis $ratio_2_axis >> $file_2_axis
done
elif [ "$object" = "beam" ]; then
file=beam_vol.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
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
cd ~
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
    cd ~/lepp3/
    sed -i "106,109s/.*/#/" artificial_pcd.toml
    sed -i "86s/.*/     depth = 0/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 3/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/lepp3/build/
    timeout 5s ./lola --cfg ../artificial_pcd.toml
    original_volume=0.0035
    echo "Original volume of $object: $original_volume"
    final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Volume after application of scale factors: $final_volume"
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_0_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_0_axis"
    diff_0_axis=$(echo $final_volume\-$est_0_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_0_axis < 0" | bc -l) )); then
    ratio_0_axis=$(echo $final_volume\/$est_0_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_0_axis=$(echo $est_0_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_0_axis"
    cd ~
    file_0_axis=data_0_axis.csv
    if [ ! -e "$file_0_axis" ]; then
    touch $file_0_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_0_Axis Ratio_0_Axis | paste -sd ' ' >> $file_0_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_0_axis $ratio_0_axis >> $file_0_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 1/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_1_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_1_axis"
    diff_1_axis=$(echo $final_volume\-$est_1_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_1_axis < 0" | bc -l) )); then
    ratio_1_axis=$(echo $final_volume\/$est_1_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_1_axis=$(echo $est_1_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_1_axis"
    cd ~
    file_1_axis=data_1_axis.csv
    if [ ! -e "$file_1_axis" ]; then
    touch $file_1_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_1_Axis Ratio_1_Axis | paste -sd ' ' >> $file_1_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_1_axis $ratio_1_axis >> $file_1_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 2/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_2_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_2_axis"
    diff_2_axis=$(echo $final_volume\-$est_2_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_2_axis < 0" | bc -l) )); then
    ratio_2_axis=$(echo $final_volume\/$est_2_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_2_axis=$(echo $est_2_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_2_axis"
    cd ~
    file_2_axis=data_2_axis.csv
    if [ ! -e "$file_2_axis" ]; then
    touch $file_2_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_2_Axis Ratio_2_Axis | paste -sd ' ' >> $file_2_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_2_axis $ratio_2_axis >> $file_2_axis
done
elif [ "$object" = "cylinder" ]; then
file=cylinder_vol.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
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
cd ~
range1=100
range2=200
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
    cd ~/lepp3/
    sed -i "106,109s/.*/#/" artificial_pcd.toml
    sed -i "86s/.*/     depth = 0/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 3/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/lepp3/build/
    timeout 5s ./lola --cfg ../artificial_pcd.toml
    original_volume=0.000883
    echo "Original volume of $object: $original_volume"
    final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Volume after application of scale factors: $final_volume"
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_0_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_0_axis"
    diff_0_axis=$(echo $final_volume\-$est_0_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_0_axis < 0" | bc -l) )); then
    ratio_0_axis=$(echo $final_volume\/$est_0_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_0_axis=$(echo $est_0_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_0_axis"
    cd ~
    file_0_axis=data_0_axis.csv
    if [ ! -e "$file_0_axis" ]; then
    touch $file_0_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_0_Axis Ratio_0_Axis | paste -sd ' ' >> $file_0_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_0_axis $ratio_0_axis >> $file_0_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 1/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_1_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_1_axis"
    diff_1_axis=$(echo $final_volume\-$est_1_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_1_axis < 0" | bc -l) )); then
    ratio_1_axis=$(echo $final_volume\/$est_1_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_1_axis=$(echo $est_1_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_1_axis"
    cd ~
    file_1_axis=data_1_axis.csv
    if [ ! -e "$file_1_axis" ]; then
    touch $file_1_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_1_Axis Ratio_1_Axis | paste -sd ' ' >> $file_1_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_1_axis $ratio_1_axis >> $file_1_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 2/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_2_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_2_axis"
    diff_2_axis=$(echo $final_volume\-$est_2_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_2_axis < 0" | bc -l) )); then
    ratio_2_axis=$(echo $final_volume\/$est_2_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_2_axis=$(echo $est_2_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_2_axis"
    cd ~
    file_2_axis=data_2_axis.csv
    if [ ! -e "$file_2_axis" ]; then
    touch $file_2_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_2_Axis Ratio_2_Axis | paste -sd ' ' >> $file_2_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_2_axis $ratio_2_axis >> $file_2_axis
done
elif [ "$object" = "triangle" ]; then
file=triangle_vol.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
  url_wrl = "triangle";
  translation = [0.0, 0.0, 0.0];
  rotation = [ 0.0, -1.5708, 1.5708 ];
  scale = [0.750000, 0.560000, 0.930000];
  velocity = [0.1, 0.1, 0.0];
});' > triangle_vol.wrl
fi;
cd ~
range1=100
range2=100
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
    cd ~/lepp3/
    sed -i "106,109s/.*/#/" artificial_pcd.toml
    sed -i "86s/.*/     depth = 0/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 3/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/lepp3/build/
    timeout 5s ./lola --cfg ../artificial_pcd.toml
    original_volume=0.000826
    echo "Original volume of $object: $original_volume"
    final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Volume after application of scale factors: $final_volume"
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_0_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_0_axis"
    diff_0_axis=$(echo $final_volume\-$est_0_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_0_axis < 0" | bc -l) )); then
    ratio_0_axis=$(echo $final_volume\/$est_0_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_0_axis=$(echo $est_0_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_0_axis"
    cd ~
    file_0_axis=data_0_axis.csv
    if [ ! -e "$file_0_axis" ]; then
    touch $file_0_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_0_Axis Ratio_0_Axis | paste -sd ' ' >> $file_0_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_0_axis $ratio_0_axis >> $file_0_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 1/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_1_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_1_axis"
    diff_1_axis=$(echo $final_volume\-$est_1_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_1_axis < 0" | bc -l) )); then
    ratio_1_axis=$(echo $final_volume\/$est_1_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_1_axis=$(echo $est_1_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_1_axis"
    cd ~
    file_1_axis=data_1_axis.csv
    if [ ! -e "$file_1_axis" ]; then
    touch $file_1_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_1_Axis Ratio_1_Axis | paste -sd ' ' >> $file_1_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_1_axis $ratio_1_axis >> $file_1_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 2/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_2_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of $object (scale factors applied): $est_2_axis"
    diff_2_axis=$(echo $final_volume\-$est_2_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_2_axis < 0" | bc -l) )); then
    ratio_2_axis=$(echo $final_volume\/$est_2_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_2_axis=$(echo $est_2_axis\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_2_axis"
    cd ~
    file_2_axis=data_2_axis.csv
    if [ ! -e "$file_2_axis" ]; then
    touch $file_2_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_2_Axis Ratio_2_Axis | paste -sd ' ' >> $file_2_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume $est_2_axis $ratio_2_axis >> $file_2_axis
done
elif [ "$object" = "sequence" ]; then
file=sequence_vol.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
  url_wrl = "box";
  translation = [0.0, 0.5, 0.0];
  rotation = [0.0, 0.0, 0.0];
  scale = [0.5, 0.5, 0.5];
  velocity = [0.1, 0.1, 0.0];
},
{
  url_wrl = "beam";
  translation = [0.0, 0.0, 0.0];
  rotation = [1.5708, 0.0, 0.0];
  scale = [0.5, 0.5, 0.5];
  velocity = [0.1, 0.1, 0.0];
});' > sequence_vol.wrl
fi;
cd ~
range1=30
range2=130
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
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 3/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    sed -i "13s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/lepp3/
    sed -i "106,109s/.*/#/" artificial_pcd.toml
    sed -i "86s/.*/     depth = 0/" artificial_pcd.toml
    cd ~/lepp3/build/
    timeout 5s ./lola --cfg ../artificial_pcd.toml
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
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    beam_est_0_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of beam (scale factors applied): $beam_est_0_axis"
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -2 eval.csv)
    box_est_0_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of box (scale factors applied): $box_est_0_axis"
    diff_beam_0_axis=$(echo $final_volume_beam\-$beam_est_0_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_beam_0_axis < 0" | bc -l) )); then
    ratio_beam_0_axis=$(echo $final_volume_beam\/$beam_est_0_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_beam_0_axis=$(echo $beam_est_0_axis\/$final_volume_beam | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated beam volume is: $diff_beam_0_axis"
    diff_box_0_axis=$(echo $final_volume_box\-$box_est_0_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_box_0_axis < 0" | bc -l) )); then
    ratio_box_0_axis=$(echo $final_volume_box\/$box_est_0_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_box_smallest=$(echo $box_est_0_axis\/$final_volume_box | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated box volume is: $diff_box_0_axis"
    cd ~
    file_0_axis=data_0_axis.csv
    if [ ! -e "$file_0_axis" ]; then
    touch $file_0_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc_Beam Beam_Est_0_Axis Beam_Ratio_0_Axis Calc_Box Box_Est_0_Axis Box_Ratio_0_Axis | paste -sd ' ' >> $file_0_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume_beam $beam_est_0_axis $ratio_beam_0_axis $final_volume_box $box_est_0_axis $ratio_box_0_axis >> $file_0_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 1/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    beam_est_1_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of beam (scale factors applied): $beam_est_1_axis"
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -2 eval.csv)
    box_est_1_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of box (scale factors applied): $box_est_1_axis"
    diff_beam_1_axis=$(echo $final_volume_beam\-$beam_est_1_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_beam_1_axis < 0" | bc -l) )); then
    ratio_beam_1_axis=$(echo $final_volume_beam\/$beam_est_1_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_beam_1_axis=$(echo $beam_est_1_axis\/$final_volume_beam | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated beam volume is: $diff_beam_1_axis"
    diff_box_1_axis=$(echo $final_volume_box\-$box_est_1_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_box_1_axis < 0" | bc -l) )); then
    ratio_box_1_axis=$(echo $final_volume_box\/$box_est_1_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_box_1_axis=$(echo $box_est_1_axis\/$final_volume_box | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated box volume is: $diff_box_1_axis"
    cd ~
    file_1_axis=data_1_axis.csv
    if [ ! -e "$file_1_axis" ]; then
    touch $file_1_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc_Beam Beam_Est_1_Axis Beam_Ratio_1_Axis Calc_Box Box_Est_1_Axis Box_Ratio_1_Axis | paste -sd ' ' >> $file_1_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume_beam $beam_est_1_axis $ratio_beam_1_axis $final_volume_box $box_est_1_axis $ratio_box_1_axis >> $file_1_axis
    cd ~/lepp3/
    sed -i "86s/.*/     depth = 2/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 6s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    beam_est_2_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of beam (scale factors applied): $beam_est_2_axis"
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -2 eval.csv)
    box_est_2_axis=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of box (scale factors applied): $box_est_2_axis"
    diff_beam_2_axis=$(echo $final_volume_beam\-$beam_est_2_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_beam_2_axis < 0" | bc -l) )); then
    ratio_beam_2_axis=$(echo $final_volume_beam\/$beam_est_2_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_beam_2_axis=$(echo $beam_est_2_axis\/$final_volume_beam | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated beam volume is: $diff_beam_2_axis"
    diff_box_2_axis=$(echo $final_volume_box\-$box_est_2_axis | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_box_2_axis < 0" | bc -l) )); then
    ratio_box_2_axis=$(echo $final_volume_box\/$box_est_2_axis | bc -l | awk '{printf "%f", $0}')
    else
    ratio_box_2_axis=$(echo $box_est_2_axis\/$final_volume_box | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated box volume is: $diff_box_2_axis"
    cd ~
    file_2_axis=data_2_axis.csv
    if [ ! -e "$file_2_axis" ]; then
    touch $file_2_axis
    fi;
    if [ "$i" == 1 ]; then
    printf '%s\n' Scale_x Scale_y Scale_z Calc_Beam Beam_Est_2_Axis Beam_Ratio_2_Axis Calc_Box Box_Est_2_Axis Box_Ratio_2_Axis | paste -sd ' ' >> $file_2_axis
    fi;
    echo $scale_x $scale_y $scale_z $final_volume_beam $beam_est_2_axis $ratio_beam_2_axis $final_volume_box $box_est_2_axis $ratio_box_2_axis >> $file_2_axis
done
else
echo "Error. Object is not defined!!!"
exit 1
fi;