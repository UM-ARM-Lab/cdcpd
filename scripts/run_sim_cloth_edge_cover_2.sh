#!/usr/bin/env bash

rosrun cdcpd_ros bagfile \
_bagfile:=cloth_edge_cover_4 \
_zeta:=2.0 \
_is_sim:=true \
_is_rope:=false \
_is_pred1:=true \
_is_pred2:=true \
_is_no_pred:=true \
_is_record:=true \
_is_gripper_info:=true \
_translation_dir_deformablity:=5.0 \
_translation_dis_deformablity:=5.0 \
_rotation_deformability:=10.0 \
_init_pt_0:=-0.19 \
_init_pt_1:=-0.19 \
_init_pt_2:=2.00 \
_init_pt_3:=-0.19 \
_init_pt_4:=0.19 \
_init_pt_5:=2.00 \
_init_pt_6:=0.19 \
_init_pt_7:=-0.19 \
_init_pt_8:=2.00 \
_cylinder_data_0:=-1 \
_cylinder_data_1:=0 \
_cylinder_data_2:=0 \
_cylinder_data_3:=100 \
_cylinder_data_4:=100 \
_cylinder_data_5:=100 \
_cylinder_data_6:=100 \
_cylinder_data_7:=-1 \
_cloth_width:=20 \
_cloth_height:=20
