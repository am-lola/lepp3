#!/bin/bash
echo "Evaluation of angle of ramp"
file=inclination.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "ramp";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.56, 0.5, 0.5 ];
});' > inclination.wrl
fi;
cd ~
store_data=angle_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Scale_z Calculated Simulated Difference Ratio | paste -sd ' ' >> $store_data
range1=50
range2=150
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x #0.54
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    R_z=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_z
    scale_z=$(echo $R_z\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_z
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 5s ./lola --cfg ../config/artificial_pcd.toml
    pi=3.14159265359
    original_slope=$(echo 64\*$scale_z\/500\/$scale_y | bc -l | awk '{printf "%f", $0}')
    original_angle=$(echo $original_slope\*180\/$pi | bc -l | awk '{printf "%f", $0}')
    echo "Original slope of ramp: $original_slope"
    echo "Original angle of ramp: $original_angle"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated angle is: $angle"
    diff=$(echo $original_angle\-$angle | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $original_angle\/$angle | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $angle\/$original_angle | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and simulated angle is: $diff"
    cd ~
    echo $scale_x $scale_y $scale_z $original_angle $angle $diff $ratio >> $store_data
done