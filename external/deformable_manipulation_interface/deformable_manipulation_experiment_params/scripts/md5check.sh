#!/usr/bin/env bash

# Usage: md5check.sh EXPERIMENT_NAME
#     E.g.: md5check.sh cloth_single_pole

EXIT_SUCCESS=0
EXIT_FAILURE=1

experiment=$1
if [[ ${experiment} == rope_generic* ]]
then
    experiment="rope_generic"
fi

params_dir=`rospack find deformable_manipulation_experiment_params`/launch/experiments
log_dir=`rosparam get log_folder`
cpp_dir=`rospack find deform_simulator`/src/custom_scene
hpp_dir=`rospack find deform_simulator`/include/custom_scene

params_files="${params_dir}/${experiment}_params.launch"
cpp_files=${cpp_dir}/*.cpp
hpp_files="${hpp_dir}/*.h ${hpp_dir}/*.hpp"

md5_file=${log_dir}/../${experiment}_environment.md5sum
mkdir -p ${log_dir}

# If the md5_file does not exist, then we need to create it, and mark it as a failed check
if [ ! -f ${md5_file} ]; then
    echo "md5sum file for ${experiment} does not exist, creating."
    md5sum ${params_files} ${cpp_files} ${hpp_files} > ${md5_file}
    exit ${EXIT_FAILURE}
else
    # Check the md5sum file
    md5parse=`md5sum -c ${md5_file}`
    if [[ ${md5parse} == *"FAILED"* ]]; then # || [[ ${md5parse} == *"No such file or directory"* ]]; then
        echo "md5sum file for ${experiment} does not match, recreating."
        md5sum ${params_files} ${cpp_files} ${hpp_files} > ${md5_file}
        exit ${EXIT_FAILURE}
    else
        exit ${EXIT_SUCCESS}
    fi
fi


