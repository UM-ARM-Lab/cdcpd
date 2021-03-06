#!/usr/bin/env bash

rosrun cdcpd_ros bagfile \
_bagfile:=cloth_cover_cylinder_6 \
_zeta:=2.0 \
_is_sim:=true \
_is_rope:=false \
_is_pred1:=true \
_is_pred2:=true \
_is_no_pred:=true \
_is_record:=false \
_is_gripper_info:=true \
_is_truth:=true \
_translation_dir_deformablity:=5.0 \
_translation_dis_deformablity:=5.0 \
_rotation_deformability:=10.0 \
_is_interaction:=true \
_init_pt_0:=0.19 \
_init_pt_1:=0.19 \
_init_pt_2:=2.00 \
_init_pt_3:=0.19 \
_init_pt_4:=-0.19 \
_init_pt_5:=2.00 \
_init_pt_6:=-0.19 \
_init_pt_7:=0.19 \
_init_pt_8:=2.00 \
_cylinder_data_0:=0 \
_cylinder_data_1:=1 \
_cylinder_data_2:=0 \
_cylinder_data_3:=0.0 \
_cylinder_data_4:=0.0 \
_cylinder_data_5:=1.82 \
_cylinder_data_6:=0.035 \
_cylinder_data_7:=0.7 \
_cylinder_quaternion_0:=0.7071068 \
_cylinder_quaternion_1:=0.0 \
_cylinder_quaternion_2:=0.0 \
_cylinder_quaternion_3:=0.7071068 \
_cloth_width:=20 \
_cloth_height:=20
