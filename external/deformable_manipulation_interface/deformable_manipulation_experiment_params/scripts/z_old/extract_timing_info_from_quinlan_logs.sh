#!/usr/bin/env bash

EXPERIMENT=${1}
TRIAL_DIR="isrr_quinlan_band_wednesday_0035"
TRIAL_PREFIX="trial_"

OUTPUT_DIR="${EXPERIMENT}/${TRIAL_DIR}"
GREP_DIR="${EXPERIMENT}/${TRIAL_DIR}/${TRIAL_PREFIX}?"

STARTING_DIR=${PWD}
cd "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/"

grep -r "total_samples"                                             ${GREP_DIR} | grep -o "[0-9]*$" > ${OUTPUT_DIR}/planning_total_samples.txt
grep -r "total_states"                                              ${GREP_DIR} | grep -o "[0-9]*$" > ${OUTPUT_DIR}/planning_total_states.txt
grep -r "planning0_sampling_time"                                   ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/planning_sampling_time.txt
grep -r "planning1_nearest_neighbour_time"                          ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/planning_nn_time.txt
grep -r "planning2_forward_propogation_band_sim_time"               ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/planning_band_sim_time.txt
grep -r "planning3_forward_propogation_first_order_vis_time"        ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/planning_first_order_vis_time.txt
grep -r "planning4_forward_propogation_everything_included_time"    ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/planning_forward_prop_everything_time.txt
grep -r "planning5_total_time"                                      ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/planning_total_time.txt
grep -r "smoothing0_failed_iterations"                              ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/smoothing_failed_iterations.txt
grep -r "smoothing1_iterations"                                     ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/smoothing_iterations_time.txt
grep -r "smoothing2_forward_propogation_band_sim_time"              ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/smoothing_band_sim_time.txt
grep -r "smoothing3_forward_propogation_first_order_vis_time"       ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/smoothing_first_ordeer_vis_time.txt
grep -r "smoothing4_forward_propogation_everything_included_time"   ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/smoothing_forward_prop_everything_time.txt
grep -r "smoothing5_total_time"                                     ${GREP_DIR} | grep -o "[0-9]*\.*[0-9]*$" > ${OUTPUT_DIR}/smoothing_total_time.txt

cd ${STARTING_DIR}