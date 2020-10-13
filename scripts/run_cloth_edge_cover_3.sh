#!/usr/bin/env bash

rosrun cdcpd_ros bagfile \
_bagfile:=cloth_edge_cover_3 \
_zeta:=2.0 \
_is_sim:=false \
_is_rope:=false \
_is_pred1:=true \
_is_pred2:=true \
_is_no_pred:=true \
_translation_dir_deformablity:=1.0 \
_translation_dis_deformablity:=1.0 \
_rotation_deformability:=10.0 \
_init_pt_0:=-0.58 \
_init_pt_1:=0.13 \
_init_pt_2:=1.2 \
_init_pt_3:=-0.58 \
_init_pt_4:=0.27 \
_init_pt_5:=0.96 \
_init_pt_6:=-0.3 \
_init_pt_7:=0.13 \
_init_pt_8:=1.2 \

