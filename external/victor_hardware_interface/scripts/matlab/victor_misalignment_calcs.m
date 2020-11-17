clc;clear;
%%
format long
x0 = [0.0   0.0   -0.863   0 0 0]
starting_error = victor_misalignment_error_fn(x0)


% T_vt_to_vr = eye(4);
% T_vt_to_vr(1:3, 4) = [x0(1); x0(2); x0(3)];
% T_vt_to_vr(1:3, 1:3) = rotx(x0(4)) * roty(x0(5)) * rotz(x0(6));
% left_arm_delta  = T_mocap_to_vt * T_vt_to_vr * T_vr_to_left * gripper_to_tc - T_mocap_to_ts * ts_to_left_tc;
% right_arm_delta = T_mocap_to_vt * T_vt_to_vr * T_vr_to_right * gripper_to_tc - T_mocap_to_ts * ts_to_right_tc;


[x_final, final_error] = fminsearch(@(x) victor_misalignment_error_fn(x), x0)
p_final_vt_to_vr = x_final(1:3)
q_final_vt_to_vr = rot2quat(rotx(x_final(4)) * roty(x_final(5)) * rotz(x_final(6)))'
T_final_vt_to_vr = trans(p_final_vt_to_vr);
T_final_vt_to_vr(1:3, 1:3) = quat2rot(q_final_vt_to_vr)
