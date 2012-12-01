# ** \file createModel.sh
# * \brief Script for creating 3D models.
# *
# * This script invokes all the programs to create a 3D model of an object
# * from a set of images and segmented images. This script can be invoked 
# * directory of by means of the ROS node createPointCloudModel:
# * $ rosrun re_vision createPointCloudModel.
# * Before using this script, you must modify the PMVS_BASE and BUNDLER_BASE
# * paths in the configuration section below.
# * 
# * This file is part of the RoboEarth ROS WP-1 package.
# * 
# * It file was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
# * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
# *
# * Copyright (C) 2011 by <a href=" mailto:dorian@unizar.es">Dorian Galvez-Lopez</a>, University of Zaragoza
# * 
# * This program is free software: you can redistribute it and/or modify
# * it under the terms of the GNU General Public License as published by
# * the Free Software Foundation, either version 3 of the License, or
# * any later version.
# *
# * This program is distributed in the hope that it will be useful,
# * but WITHOUT ANY WARRANTY; without even the implied warranty of
# * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# * GNU General Public License for more details.
# *
# * You should have received a copy of the GNU General Public License
# * along with this program.  If not, see <http://www.gnu.org/licenses/>.
# *
# * \author Dorian Galvez-Lopez
# * \version 1.0
# * \date 2011
# * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
# * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
#***********************************************/

# --- Configuration ---------------------------------------------------------

PMVS_BASE=/opt/pmvs-2
BUNDLER_BASE=/opt/bundler-v0.4-source

# ---------------------------------------------------------------------------

# exit on error and use set variables
set -e
set -u

if [ $# -le 3 ]; then
  echo "Usage: $0 <image_dir> <image_model_dir> <name> <model_out_dir> <surf_threshold>"
  exit 0
fi

SEGMENTED_IMAGE_DIR="$1"
SEGMENTED_OBJECT_DIR="$2"
OBJECT_NAME="$3"
MODEL_OUT_DIR="$4"
SURF_THRESHOLD="$5"

# create a temporal directory with the bundle and pmvs files
CUR_DIR=`pwd`
AUX_DIR=.bundle_`date "+%Y%m%d_%H%M%S"`

echo "[- Creating temporal directory $AUX_DIR -]"

mkdir $AUX_DIR

# run bundler

BASE_PATH=$BUNDLER_BASE
BIN_PATH=$BASE_PATH/bin
BUNDLER=$BIN_PATH/bundler
BUNDLE2PMVS=$BIN_PATH/Bundle2PMVS
PMVS_BIN_PATH="$PMVS_BASE/program/main"
PMVS=$PMVS_BIN_PATH/pmvs2
EXTRACT_FOCAL=$BASE_PATH/bin/extract_focal.pl

MATCHKEYS=$BUNDLER_BASE/bin/KeyMatchFull
TO_SIFT=$BUNDLER_BASE/bin/ToSift.sh

# Rename ".JPG" to ".jpg"
for d in `ls -1 $SEGMENTED_IMAGE_DIR | egrep ".JPG$"`
do 
    mv $SEGMENTED_IMAGE_DIR/$d $SEGMENTED_IMAGE_DIR/`echo $d | sed 's/\.JPG/\.jpg/'`
done

# Create the list of images
echo "[- Extracting focal length and distortion parameters -]"
mkdir -p prepare

echo -n "" > $AUX_DIR/list_tmp.txt
for s in `find $SEGMENTED_IMAGE_DIR -maxdepth 1 | egrep ".jpg$" | sort`; do
  readlink -f $s >> $AUX_DIR/list_tmp.txt
done
$EXTRACT_FOCAL $AUX_DIR/list_tmp.txt > /dev/null

cp prepare/list.txt $AUX_DIR
rm prepare/list.txt
rmdir prepare


# Run the ToSift script to generate a list of SIFT commands
echo "[- Extracting keypoints -]"
$TO_SIFT "$SEGMENTED_IMAGE_DIR" > "$AUX_DIR/sift.txt"

# Execute the SIFT commands
sh $AUX_DIR/sift.txt

# Match images 
echo "[- Matching keypoints -]"
sed 's/\.jpg$/\.key/' $AUX_DIR/list_tmp.txt > $AUX_DIR/list_keys.txt
$MATCHKEYS $AUX_DIR/list_keys.txt $AUX_DIR/matches.init.txt

# Generate the options file for running bundler 

echo "--match_table $CUR_DIR/$AUX_DIR/matches.init.txt" >> $AUX_DIR/options.txt
echo "--output bundle.out" >> $AUX_DIR/options.txt
echo "--output_all bundle_" >> $AUX_DIR/options.txt
echo "--output_dir $CUR_DIR/$AUX_DIR" >> $AUX_DIR/options.txt
echo "--variable_focal_length" >> $AUX_DIR/options.txt
echo "--use_focal_estimate" >> $AUX_DIR/options.txt
echo "--constrain_focal" >> $AUX_DIR/options.txt
echo "--constrain_focal_weight 0.0001" >> $AUX_DIR/options.txt
echo "--estimate_distortion" >> $AUX_DIR/options.txt
echo "--run_bundle" >> $AUX_DIR/options.txt

# Run Bundler!
echo "[- Running Bundler -]" 
# change the directory because bundler creates intermediate files in pwd
cd $AUX_DIR 
$BUNDLER $CUR_DIR/$AUX_DIR/list.txt --options_file $CUR_DIR/$AUX_DIR/options.txt > $CUR_DIR/$AUX_DIR/out
cd $CUR_DIR

echo "[- Done -]"

# Run Bundle2PMVS
echo "[- Running Bundle2PMVS -]"
BUNDLE_OUT_FILE="$AUX_DIR/bundle.out"
PMVS_OUT_DIR="$AUX_DIR/pmvs"
$BUNDLE2PMVS $AUX_DIR/list.txt $BUNDLE_OUT_FILE $PMVS_OUT_DIR > $AUX_DIR/.bundle2pmvs.out
sh $PMVS_OUT_DIR/prep_pmvs.sh "$BIN_PATH" 0 > $AUX_DIR/.prep_pmvs.out
# 0 means using the images given to this script (segmented)
# 1 means using the images with background

echo "[- Done -]"

# Run PMVS
echo "[- Running PMVS -]"
$PMVS $PMVS_OUT_DIR/ pmvs_options.txt > $PMVS_OUT_DIR/.pmvs.out 2> $PMVS_OUT_DIR/.pmvs.2.out
echo "[- Done -]"

# Copy the model files
echo "[- Creating image and mask files -]"
mkdir -p "$MODEL_OUT_DIR"

n=0
for s in `ls $PMVS_OUT_DIR/visualize/*.jpg`
do
  cp $s `printf "$MODEL_OUT_DIR/im%02d.jpg" $n`
  n=`expr $n + 1`
done

# create the mask files
./extractOutlineFromImages "$PMVS_OUT_DIR/list.rd.txt" "$MODEL_OUT_DIR" "$SEGMENTED_OBJECT_DIR"

echo "[- Done -]"

# Get pixel points
echo "[- Getting dense points -]"

./removeBackgroundPoints \
  "$PMVS_OUT_DIR/models/pmvs_options.txt.ply" \
  "$PMVS_OUT_DIR/models/pmvs_options.txt.patch" \
  "$PMVS_OUT_DIR/txt/" \
  "$MODEL_OUT_DIR" \
  "$PMVS_OUT_DIR/models/pmvs_options.txt_nobg.ply" \
  "$PMVS_OUT_DIR/models/pmvs_options.txt_nobg.patch"

mkdir -p "$PMVS_OUT_DIR/txt_changed/"
./changeReferenceSystem \
  "$PMVS_OUT_DIR/txt/" \
  "$PMVS_OUT_DIR/models/pmvs_options.txt_nobg.ply" \
  "$PMVS_OUT_DIR/models/pmvs_options.txt_nobg.patch" \
  "$PMVS_OUT_DIR/txt_changed/" \
  "$MODEL_OUT_DIR/dense_model.ply" \
  "$MODEL_OUT_DIR/dense_model.patch"

# computePixelPoints produces PMVS cameras
# computePixelPoints2 produces Bundle cameras
./computePixelPoints2 \
  "$MODEL_OUT_DIR/dense_model.ply" \
  "$MODEL_OUT_DIR/dense_model.patch" \
  "$MODEL_OUT_DIR" \
  "$PMVS_OUT_DIR/txt_changed/"

echo "[- Done -]"

## This was of a previous version
#
## Get the fast points
#echo "[- Getting dense FAST points with threshold $FAST_THRESHOLD -]"
#./extractFast "$MODEL_OUT_DIR" "$FAST_THRESHOLD"
#echo "[- Done -]"
#
## and calculate the correspondences
#echo "[- Computing correspondences -]"
#./computeCorrespondences "$MODEL_OUT_DIR"
#echo "[- Done -]"
#

# Get the surf points
echo "[- Getting dense SURF points with threshold $SURF_THRESHOLD -]"
./extractSurf "$MODEL_OUT_DIR" "$SURF_THRESHOLD"
echo "[- Done -]"

echo "[- Creating final PLY points -]"
./createPlyPoints "$MODEL_OUT_DIR"
echo "[- Done -]"

echo "[- Generating final model files -]"

## change cp for mv
mv "$MODEL_OUT_DIR/dense_model.ply" "$MODEL_OUT_DIR/drawing_model.ply"
rm "$MODEL_OUT_DIR/dense_model.patch"

# number of faces
N=`find $MODEL_OUT_DIR -iname "*.jpg" | wc -l`
for i in `seq 0 $(($N - 1))`
do
  convert `printf "$MODEL_OUT_DIR/im%02d.jpg" $i` `printf "$MODEL_OUT_DIR/face_%03d.png" $i`
  
  mv `printf "$MODEL_OUT_DIR/im%02d.key.gz" $i` `printf "$MODEL_OUT_DIR/face_%03d.key.gz" $i`
  mv `printf "$MODEL_OUT_DIR/im%02d.ply" $i` `printf "$MODEL_OUT_DIR/face_%03d.ply" $i`
  mv `printf "$MODEL_OUT_DIR/im%02d_cam.txt" $i` `printf "$MODEL_OUT_DIR/face_%03d.txt" $i`

  rm `printf "$MODEL_OUT_DIR/im%02d.jpg" $i`
  rm `printf "$MODEL_OUT_DIR/im%02d_mask.png" $i`
  rm `printf "$MODEL_OUT_DIR/im%02d_2d3d.txt" $i`
  rm `printf "$MODEL_OUT_DIR/im%02d_points.txt" $i`
done

# generate xml
SCALE_FACTOR=1 # models up to scale
./generateMeta "$MODEL_OUT_DIR/meta.xml" "$OBJECT_NAME" $N $SCALE_FACTOR

rm -r $AUX_DIR

echo "[- Done -]"


