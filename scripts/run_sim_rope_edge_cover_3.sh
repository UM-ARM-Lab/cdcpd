#!/usr/bin/env bash

rosrun cdcpd_ros bagfile \
_bagfile:=rope_edge_cover_3 \
_alpha:=0.5 \
_lambda:=1.0 \
_zeta:=2.0 \
_is_sim:=true \
_is_rope:=true \
_is_pred1:=false \
_is_pred2:=false \
_is_no_pred:=true \
_is_record:=true \
_is_gripper_info:=false \
_translation_dir_deformablity:=5.0 \
_translation_dis_deformablity:=5.0 \
_rotation_deformablity:=10.0 \
_rope_points:= 50 \
_init_pt_0:=-1.0 \
_init_pt_1:=0.0 \
_init_pt_2:=3.0 \
_init_pt_3:=0.0 \
_init_pt_4:=0.0 \
_init_pt_5:=3.0 \
_init_pt_6:=-0.20 \
_init_pt_7:=0.08 \
_init_pt_8:=1.3 \
_cylinder_data_0:=-1.0 \
_cylinder_data_1:=-1.0 \
_cylinder_data_2:=-1.0 \
_cylinder_data_3:=-1.0 \
_cylinder_data_4:=-1.0 \
_cylinder_data_5:=-1.0 \
_cylinder_data_6:=-1.0 \
_cylinder_data_7:=-1.0
