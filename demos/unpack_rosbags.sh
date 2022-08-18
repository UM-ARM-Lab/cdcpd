#!/bin/bash

DEMO_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
INPUT_DIR="${DEMO_DIR}/rosbags_compressed"
OUTPUT_DIR="${DEMO_DIR}/rosbags"
#ROSBAGS_DIR="${DEMO_DIR}/rosbags"
#ROSBAG_FILES="${ROSBAGS_DIR}/*"
#echo $INPUT_DIR
#ls $INPUT_DIR

for f in $(ls $INPUT_DIR); do
	echo "Unpacking $f"
	rosbag decompress "${INPUT_DIR}/${f}" --output-dir=$OUTPUT_DIR
done
