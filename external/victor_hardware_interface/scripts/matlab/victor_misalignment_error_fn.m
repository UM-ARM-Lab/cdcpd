function [error_sq] = victor_misalignment_error_fn(x)
% seq: 0
% stamp: 
%   secs: 1505241831
%   nsecs: 460365057
% frame_id: mocap_world
% mocap_VictorTorso_VictorTorso
% Position (xyz):     1.80084053901   -0.201312212515   0.251204525229
% Quaternion (xyzw):  0.00458177080242   -0.0686695179642   0.00662349846398   0.997606953638
p_mocap_to_vt = [1.80084053901   -0.201312212515   0.251204525229]';
q_mocap_to_vt = [0.00458177080242   -0.0686695179642   0.00662349846398   0.997606953638]';
T_mocap_to_vt = trans(p_mocap_to_vt);
T_mocap_to_vt(1:3, 1:3) = quat2rot(q_mocap_to_vt);

% seq: 0
% stamp: 
%   secs: 1505242103
%   nsecs: 873762846
% frame_id: mocap_world
% table_surface
% Position (xyz):     2.62936870417   -0.195548308333   0.412984736524
% Quaternion (xyzw):  0.00145586147201   -0.068183833476   0.0331161011276   0.997121942977
p_mocap_to_ts = [2.62936870417   -0.195548308333   0.412984736524];
q_mocap_to_ts = [0.00145586147201   -0.068183833476   0.0331161011276   0.997121942977];
T_mocap_to_ts = trans(p_mocap_to_ts);
T_mocap_to_ts(1:3, 1:3) = quat2rot(q_mocap_to_ts);


% seq: 0
% stamp: 
%   secs: 1505242145
%   nsecs: 226448059
% frame_id: victor_root
% victor_left_gripper_palm_surface
% Position (xyz):     0.371337124263   0.581507177542   0.926908227338
% Quaternion (xyzw):  0.626232620143   -0.364796073193   0.652787729627   0.220510114283
p_vr_to_left = [0.371337124263   0.581507177542   0.926908227338]';
q_vr_to_left = [0.626232620143   -0.364796073193   0.652787729627   0.220510114283]';
T_vr_to_left = trans(p_vr_to_left);
T_vr_to_left(1:3, 1:3) = quat2rot(q_vr_to_left);

% seq: 0
% stamp: 
%   secs: 1505242168
%   nsecs: 226422071
% frame_id: victor_root
% victor_right_gripper_palm_surface
% Position (xyz):     0.395312597643   -0.654428423288   0.943905006658
% Quaternion (xyzw):  0.648914792544   0.432731797658   0.611009974842   -0.13534989456
p_vr_to_right = [0.395312597643   -0.654428423288   0.943905006658];
q_vr_to_right = [0.648914792544   0.432731797658   0.611009974842   -0.13534989456];
T_vr_to_right = trans(p_vr_to_right);
T_vr_to_right(1:3, 1:3) = quat2rot(q_vr_to_right);

gripper_to_table_corner = [0 0 0.117475 1]';
ts_to_left_table_corner = [-0.381 0.5334 0 1]';
ts_to_right_table_corner = [-0.381 -0.5334 0 1]';


T_vt_to_vr = eye(4);
T_vt_to_vr(1:3, 4) = [x(1); x(2); x(3)];
T_vt_to_vr(1:3, 1:3) = rotx(x(4)) * roty(x(5)) * rotz(x(6));
left_arm_delta  = T_mocap_to_vt * T_vt_to_vr * T_vr_to_left * gripper_to_table_corner - T_mocap_to_ts * ts_to_left_table_corner;
right_arm_delta = T_mocap_to_vt * T_vt_to_vr * T_vr_to_right * gripper_to_table_corner - T_mocap_to_ts * ts_to_right_table_corner;


error_sq = sum(left_arm_delta.^2 + right_arm_delta.^2);

end

