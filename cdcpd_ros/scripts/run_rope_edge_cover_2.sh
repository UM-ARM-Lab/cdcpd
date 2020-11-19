#!/usr/bin/env bash

rosrun cdcpd_ros bagfile \
_bagfile:=../dataset/rope_edge_cover_2 \
_zeta:=2.0 \
_is_sim:=false \
_is_rope:=true \
_is_pred1:=true \
_is_pred2:=true \
_is_no_pred:=true \
_is_gripper_info:=true \
_translation_dir_deformablity:=5.0 \
_translation_dis_deformablity:=5.0 \
_rotation_deformablity:=10.0 \
_init_pt_0:=-0.7 \
_init_pt_1:=0.0 \
_init_pt_2:=1.0 \
_init_pt_3:=0.1 \
_init_pt_4:=0.0 \
_init_pt_5:=1.0 \
_init_pt_6:=-0.20 \
_init_pt_7:=0.08 \
_init_pt_8:=1.3 \

