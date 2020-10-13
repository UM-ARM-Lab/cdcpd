#!/usr/bin/env bash

rosrun cdcpd_ros bagfile \
_bagfile:=rope_by_hand_6_comp \
_zeta:=2.0 \
_is_sim:=false \
_is_rope:=true \
_is_pred1:=false \
_is_pred2:=false \
_is_no_pred:=true \
_is_record:=false \
_is_gripper_info:=false \
_is_interaction:=false \
_translation_dir_deformablity:=5.0 \
_translation_dis_deformablity:=5.0 \
_rotation_deformablity:=10.0 \
_rope_points:=50 \
_init_pt_0:=-0.55 \
_init_pt_1:=-0.16 \
_init_pt_2:=1.5 \
_init_pt_3:=0.4 \
_init_pt_4:=-0.25 \
_init_pt_5:=1.5 \
_init_pt_6:=-0.20 \
_init_pt_7:=0.08 \
_init_pt_8:=1.3 \

