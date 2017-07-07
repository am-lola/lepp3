#!/bin/bash
object="object"
echo -n -e "Enter the object you want to evaluate\n"
read object
echo -e "Evaluation of $object :\n"
if [ "$object" = "box" ]; then
scale=0.1
echo -n -e "Enter scale factor\n"
read scale
file=box_veloc.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
 {
 	url_wrl = "box";
 	translation = [0.0, 0.0, 0.0];
 	rotation = [0.0, 0.0, 0.0];
  scale = [0.5, 0.5, 0.5];
  velocity = [0.170000, 0.110000, 0.130000];
 });' > box_veloc.wrl
 fi;
cd ~
store_data=velocity_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Veloc_x Simul_Veloc_x Diff_x Ratio_x Veloc_y Simul_Veloc_y Diff_y Ratio_y Veloc_z Simul_Veloc_z Diff_z Ratio_z | paste -sd ' ' >> $store_data
range1=10
range2=20
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%DIFF))+$range1))
    echo $R_x
    x_velocity=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $x_velocity
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    y_velocity=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $y_velocity
    R_z=$(($(($RANDOM%DIFF))+$range1))
    echo $R_z
    z_velocity=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
    echo $z_velocity
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "7s/.*/  velocity = [$x_velocity, $y_velocity, $z_velocity]; /" "$file"
    sed -i "6s/.*/  scale = [$scale, $scale, $scale]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" --stream 10
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 5s ./lola --cfg ../config/artificial_stream.toml
    cd ~/Music/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    diff_x=$(echo $x_velocity - $sim_veloc_x | bc -l | awk '{printf "%f", $0}')
    diff_y=$(echo $y_velocity - $sim_veloc_y | bc -l | awk '{printf "%f", $0}')
    diff_z=$(echo $z_velocity - $sim_veloc_z | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_x < 0" | bc -l) )); then
    ratio_x=$(echo $x_velocity\/$sim_veloc_x | bc -l | awk '{printf "%f", $0}')
    else
    ratio_x=$(echo $sim_veloc_x\/$x_velocity | bc -l | awk '{printf "%f", $0}')
    fi;
    if (( $(echo "$diff_y < 0" | bc -l) )); then
    ratio_y=$(echo $y_velocity\/$sim_veloc_y | bc -l | awk '{printf "%f", $0}')
    else
    ratio_y=$(echo $sim_veloc_y\/$y_velocity | bc -l | awk '{printf "%f", $0}')
    fi;
    if (( $(echo "$diff_z < 0" | bc -l) )); then
    ratio_z=$(echo $z_velocity\/$sim_veloc_z | bc -l | awk '{printf "%f", $0}')
    else
    ratio_z=$(echo $sim_veloc_z\/$z_velocity | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between real and simulated x velocity is: $diff_x"
    echo "The difference between real and simulated y velocity is: $diff_y"
    echo "The difference between real and simulated z velocity is: $diff_z"
    cd ~
    echo $x_velocity $sim_veloc_x $diff_x $ratio_x $y_velocity $sim_veloc_y $diff_y $ratio_y $z_velocity $sim_veloc_z $diff_z $ratio_z >> $store_data
done
elif [ "$object" = "cylinder" ]; then
scale=0.1
echo -n -e "Enter scale factor\n"
read scale
file=cylinder_veloc.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
 {
 	url_wrl = "cylinder";
 	translation = [0.0, 0.0, 0.0];
 	rotation = [0.0, 0.0, 0.0];
  scale = [0.5, 0.5, 0.5];
  velocity = [0.170000, 0.110000, 0.130000];
 });' > cylinder_veloc.wrl
 fi;
cd ~
store_data=velocity_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Veloc_x Simul_Veloc_x Diff_x Ratio_x Veloc_y Simul_Veloc_y Diff_y Ratio_y Veloc_z Simul_Veloc_z Diff_z Ratio_z | paste -sd ' ' >> $store_data
range1=10
range2=100
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%DIFF))+$range1))
    echo $R_x
    x_velocity=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $x_velocity
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    y_velocity=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $y_velocity
    R_z=$(($(($RANDOM%DIFF))+$range1))
    echo $R_z
    z_velocity=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
    echo $z_velocity
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "7s/.*/  velocity = [$x_velocity, $y_velocity, $z_velocity]; /" "$file"
    sed -i "6s/.*/  scale = [$scale, $scale, $scale]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" --stream 10
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 5s ./lola --cfg ../config/artificial_stream.toml
    cd ~/Music/lepp3/evaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r model_id volume sim_veloc_x sim_veloc_y sim_veloc_z < <(tail -1 eval.csv)
    diff_x=$(echo $x_velocity - $sim_veloc_x | bc -l | awk '{printf "%f", $0}')
    diff_y=$(echo $y_velocity - $sim_veloc_y | bc -l | awk '{printf "%f", $0}')
    diff_z=$(echo $z_velocity - $sim_veloc_z | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_x < 0" | bc -l) )); then
    ratio_x=$(echo $x_velocity\/$sim_veloc_x | bc -l | awk '{printf "%f", $0}')
    else
    ratio_x=$(echo $sim_veloc_x\/$x_velocity | bc -l | awk '{printf "%f", $0}')
    fi;
    if (( $(echo "$diff_y < 0" | bc -l) )); then
    ratio_y=$(echo $y_velocity\/$sim_veloc_y | bc -l | awk '{printf "%f", $0}')
    else
    ratio_y=$(echo $sim_veloc_y\/$y_velocity | bc -l | awk '{printf "%f", $0}')
    fi;
    if (( $(echo "$diff_z < 0" | bc -l) )); then
    ratio_z=$(echo $z_velocity\/$sim_veloc_z | bc -l | awk '{printf "%f", $0}')
    else
    ratio_z=$(echo $sim_veloc_z\/$z_velocity | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between real and simulated x velocity is: $diff_x"
    echo "The difference between real and simulated y velocity is: $diff_y"
    echo "The difference between real and simulated z velocity is: $diff_z"
    cd ~
    echo $x_velocity $sim_veloc_x $diff_x $ratio_x $y_velocity $sim_veloc_y $diff_y $ratio_y $z_velocity $sim_veloc_z $diff_z $ratio_z >> $store_data
done
fi;