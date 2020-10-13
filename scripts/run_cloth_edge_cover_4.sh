#!/usr/bin/env bash

rosrun cdcpd_ros bagfile \
_bagfile:=cloth_edge_cover_4 \
_zeta:=2.0 \
_is_sim:=false \
_is_rope:=false \
_is_pred1:=true \
_is_pred2:=true \
_is_no_pred:=true \
_is_record:=true \
_translation_dir_deformablity:=5.0 \
_translation_dis_deformablity:=5.0 \
_rotation_deformability:=10.0 \
_init_pt_0:=0.22 \
_init_pt_1:=-0.2 \
_init_pt_2:=1.25 \
_init_pt_3:=0.22 \
_init_pt_4:=-0.02 \
_init_pt_5:=1.08 \
_init_pt_6:=0.5 \
_init_pt_7:=-0.2 \
_init_pt_8:=1.25 \
_cylinder_data_0:=-1 \
_cylinder_data_1:=0 \
_cylinder_data_2:=0 \
_cylinder_data_3:=100 \
_cylinder_data_4:=100 \
_cylinder_data_5:=100 \
_cylinder_data_6:=100 \
_cylinder_data_7:=-1 \
_cloth_width:=15 \
_cloth_height:=15
