#!/bin/bash
object="object"
echo -n -e "Enter the object you want to evaluate\n"
read object
echo -e "Evaluation of $object :\n"
if [ "$object" = "floor" ]; then
file=floor_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "floor";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > floor_surf.wrl
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
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 40s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=40.130005
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "platform" ]; then
file=platform_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "platform";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > platform_surf.wrl
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
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 20s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=2.25
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "stair" ]; then
file=stair_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "stair";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > stair_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=20
range2=110
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 15s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=1.01
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "board" ]; then
file=board_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "board";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > board_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=60
range2=150
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 6s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=0.09
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "pentagon" ]; then
file=pentagon_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "pentagon";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > pentagon_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=60
range2=150
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 20s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=1.018750
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "pentagon_inclined" ]; then
file=pentagon_inclined_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "pentagon_inclined";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > pentagon_inclined_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=60
range2=150
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 20s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=1.024428
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "irregular" ]; then
file=irregular_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "irregular";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > irregular_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=60
range2=150
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 10s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=1.840635
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "irregular_inclined" ]; then
file=irregular_inclined_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "irregular_inclined";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > irregular_inclined_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=60
range2=150
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 10s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=1.856843
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "round" ]; then
file=round_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "round";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > round_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=60
range2=150
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 3/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 6s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=0.384772
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "round_inclined" ]; then
file=round_inclined_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
 url_wrl = "round_inclined";
 translation = [ 0.0, 0.0, 0.0 ];
 rotation = [ 0.0, 0.0, 0.0 ];
 scale = [ 0.5, 0.5, 0.1 ];
});' > round_inclined_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calculated Estimated Difference Ratio | paste -sd ' ' >> $store_data
range1=60
range2=150
DIFF=$(($range2-$range1+1))
RANDOM=$$
for i in `seq 20`
do
    R_x=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_x
    scale_x=$(echo $R_x\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_x
    R_y=$(($(($RANDOM%$DIFF))+$range1))
    echo $R_y
    scale_y=$(echo $R_y\/100 | bc -l | awk '{printf "%f", $0}')
    echo $scale_y
    scale_z=0.1
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 1/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 6s ./lola --cfg ../config/artificial_pcd.toml
    original_surface=0.392990
    echo "Original surface of $object: $original_surface"
    final_surface=$(echo $original_surface\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Surface after application of scale factors: $final_surface"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated surface is: $total_area"
    diff=$(echo $final_surface\-$area | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff < 0" | bc -l) )); then
    ratio=$(echo $final_surface\/$area | bc -l | awk '{printf "%f", $0}')
    else
    ratio=$(echo $area\/$final_surface | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated surface is: $diff"
    cd ~
    echo $scale_x $scale_y $final_surface $area $diff $ratio >> $store_data
done
elif [ "$object" = "sequence" ]; then
file=sequence_surf.wrl
cd ~/am2b/etc/model/pcd_creation/lab_scene/
if [ ! -e "$file" ] ; then
echo 'objects = (
{
  url_wrl = "floor";
  translation = [0.0, -1.5, 0.0];
  rotation = [0.0, 0.0, 0.0];
  scale = [0.710000, 0.280000, 0.1];
 },
 {
 	url_wrl = "platform";
 	translation = [0.0, 1.5, 0.0];
 	rotation = [0.0, 0.0, 0.0];
  scale = [0.710000, 0.280000, 0.1];
 });' > sequence_surf.wrl
fi;
cd ~
store_data=surface_data.csv
if [ ! -e "$store_data" ]; then
touch $store_data
fi;
printf '%s\n' Scale_x Scale_y Calc_Floor Est_Floor Diff_Floor Ratio_Floor Calc_Platform Est_Platform Diff_Platform Ratio_Platform | paste -sd ' ' >> $store_data
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
    scale_z=0.3
    cd ~/Music/lepp3/config/
    sed -i "124s/.*/[[observers]]/" artificial_pcd.toml
    sed -i "125s/.*/    type = \"SurfaceDetector\"/" artificial_pcd.toml
    sed -i "126s/.*/    [[aggregators]]/" artificial_pcd.toml
    sed -i "127s/.*/    type = \"SurfaceEvaluator\"/" artificial_pcd.toml
    #cd ../../../am2b/etc/model/pcd_creation/lab_scene
    cd ~/am2b/etc/model/pcd_creation/
    sed -i "3s/.*/-0.5 0 3/" pov.txt
    cd ~/am2b/etc/model/pcd_creation/lab_scene/
    sed -i "6s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    sed -i "12s/.*/  scale = [$scale_x, $scale_y, $scale_z]; /" "$file"
    cd ~/am2b/etc/model/pcd_creation/build/
    ./pcd_creator -f "$file" -p
    #cd ../../../../../lepp3/build
    cd ~/Music/lepp3/build/
    timeout 40s ./lola --cfg ../config/artificial_pcd.toml
    original_surface_floor=40.0
    original_surface_platform=2.25
    echo "Original surface of floor: $original_surface_floor"
    echo "Original surface of platform: $original_surface_platform"
    final_surface_floor=$(echo $original_surface_floor\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    final_surface_platform=$(echo $original_surface_platform\*$scale_x\*$scale_y | bc -l | awk '{printf "%f", $0}')
    echo "Scale factor x: $scale_x"
    echo "Scale factor y: $scale_y"
    echo "Scale factor z: $scale_z"
    echo "Floor surface after application of scale factors: $final_surface_floor"
    echo "Platform surface after application of scale factors: $final_surface_platform"
    cd ~/Music/lepp3/surfaceEvaluation/
    fn=$(ls -t | head -n1)
    cd $fn
    IFS=, read -r id area_floor total_area angle < <(tail -2 surfeval.csv)
    echo "Estimated floor surface is: $area_floor"
    diff_floor=$(echo $final_surface_floor\-$area_floor | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_floor < 0" | bc -l) )); then
    ratio_floor=$(echo $final_surface_floor\/$area_floor | bc -l | awk '{printf "%f", $0}')
    else
    ratio_floor=$(echo $area_floor\/$final_surface_floor | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated floor surface is: $diff_floor"
    IFS=, read -r id area_platform total_area angle < <(tail -1 surfeval.csv)
    echo "Estimated platform surface is: $area_platform"
    diff_platform=$(echo $final_surface_platform\-$area_platform | bc -l | awk '{printf "%f", $0}')
    if (( $(echo "$diff_platform < 0" | bc -l) )); then
    ratio_platform=$(echo $final_surface_platform\/$area_platform | bc -l | awk '{printf "%f", $0}')
    else
    ratio_platform=$(echo $area_platform\/$final_surface_platform | bc -l | awk '{printf "%f", $0}')
    fi;
    echo "The difference between calculated and estimated platform surface is: $diff_platform"
    cd ~
    echo $scale_x $scale_y $final_surface_floor $area_floor $diff_floor $ratio_floor $final_surface_platform $area_platform $diff_platform $ratio_platform >> $store_data
done
else
echo "Error. Object is not defined!!!"
exit 1
fi;