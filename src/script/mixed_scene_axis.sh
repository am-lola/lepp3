#!/bin/bash
splitting_step=1
echo -n -e "Enter the number of splitting steps\n"
read splitting_step
cd ~/lepp3/
sed -i "86s/.*/     depth = $splitting_step/" artificial_pcd.toml
file=floorbox.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
  url_wrl = "floor";
  translation = [0.0, -0.5, 0.0];
  rotation = [0.0, 0.0, 0.0];
  scale = [0.710000, 0.280000, 0.1];
 },
 {
 	url_wrl = "box";
 	translation = [0.0, 0.5, 0.0];
 	rotation = [0.0, 0.0, 0.0];
  scale = [0.710000, 0.280000, 0.1];
 });' > floorbox.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=10
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
    R_z=$(($(($RANDOM%$DIFF))+$range1))
    scale_z=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
    cd ~/lepp3/
    sed -i "77s/.*/split_axis = \"smallest\"/" artificial_pcd.toml
    sed -i "106s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "107s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "108s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "109s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 3/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, 0.1]; /" "$file"
    sed -i "12s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/lepp3/build/
    timeout 40s ./lola --cfg ../artificial_pcd.toml
    original_surface=40.0
    original_volume=0.016
    echo "Original surface of floor: $original_surface"
    echo "Original surface of platform: $original_volume"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    final_volume=$(echo $original_volume\*$scale_x\*$scale_y\*$scale_z | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Floor surface after application of scale factors: $final_surface"
    echo "Box volume after application of scale factors: $final_volume"
    cd ~/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area_floor total_area angle < <(tail -2 surfeval.csv)
    echo "Estimated floor surface is: $area_floor"
    diff_floor=$(echo $final_surface\-$area_floor | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_floor < 0" | bc -l) )); then
    ratio_floor=$(echo $final_surface\/$area_floor | bc -l | awk '{printf "%f", $0}')
    else
    ratio_floor=$(echo $area_floor\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated floor surface is: $diff_floor"
    cd ~/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    est_smallest=$(echo $volume\/1000000 | bc -l | awk '{printf "%f", $0}')
    echo "Estimated volume of box (scale factors applied): $est_smallest"
    diff_smallest=$(echo $final_volume\-$est_smallest | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_smallest < 0" | bc -l) )); then
    ratio_smallest=$(echo $final_volume\/$est_smallest | bc -l | awk '{printf "%f", $0}')
    else
    ratio_smallest=$(echo $est_smallest\/$final_volume | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated volume is: $diff_smallest"
    cd ~
    file_smallest=data_smallest.csv
    if [ ! -e "$file_smallest" ]; then
    touch $file_smallest
    fi;
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Smallest Ratio_Smallest | paste -sd ' ' >> $file_smallest
    echo $scale_x $scale_y $scale_z $final_volume $est_smallest $ratio_smallest >> $file_smallest
    cd ~/lepp3/
    sed -i "77s/.*/split_axis = \"middle\"/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 40s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
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
    cd ~
    file_middle=data_middle.csv
    if [ ! -e "$file_middle" ]; then
    touch $file_middle
    fi;
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Middle Ratio_Middle | paste -sd ' ' >> $file_middle
    echo $scale_x $scale_y $scale_z $final_volume $est_middle $ratio_middle >> $file_middle
    cd ~/lepp3/
    sed -i "77s/.*/split_axis = \"largest\"/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    cd ~/lepp3/build/
    timeout 40s ./lola --cfg ../artificial_pcd.toml
    cd ~/lepp3/evaluation/
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
    cd ~
    file_largest=data_largest.csv
    if [ ! -e "$file_largest" ]; then
    touch $file_largest
    fi;
    printf '%s\n' Scale_x Scale_y Scale_z Calc Est_Largest Ratio_Largest | paste -sd ' ' >> $file_largest
    echo $scale_x $scale_y $scale_z $final_volume $est_largest $ratio_largest >> $file_largest
    echo $scale_x $scale_y $final_surface $area_floor $diff_floor $ratio_floor >> $store_data
done