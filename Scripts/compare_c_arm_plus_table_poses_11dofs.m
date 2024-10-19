clc;
clear all;
close all;
%% file paths
csv_path_1 = "../Sample_data_files/c_arm_plus_table_simscape_out_poses.csv";
csv_path_2 = "../Sample_data_files/c_arm_plus_table_dh_parameters_out_poses.csv";
out_file = "../Sample_data_files/compare_simscape_dh_out.txt";

%% read poses data
data_table_1 = readtable(csv_path_1);
data_table_2 = readtable(csv_path_2);

%% table to array - quaternions
data_quat_w_1 = data_table_1.quat_w;
data_quat_x_1 = data_table_1.quat_x;
data_quat_y_1 = data_table_1.quat_y;
data_quat_z_1 = data_table_1.quat_z;
q1 = cat(2, data_quat_w_1, data_quat_x_1, data_quat_y_1, data_quat_z_1);

data_quat_w_2 = data_table_2.quat_w;
data_quat_x_2 = data_table_2.quat_x;
data_quat_y_2 = data_table_2.quat_y;
data_quat_z_2 = data_table_2.quat_z;
q2 = cat(2, data_quat_w_2, data_quat_x_2, data_quat_y_2, data_quat_z_2);

%% table to array - translations
data_trans_x_1 = data_table_1.Tx_m;
data_trans_y_1 = data_table_1.Ty_m;
data_trans_z_1 = data_table_1.Tz_m;
t1 = cat(2, data_trans_x_1, data_trans_y_1, data_trans_z_1);

data_trans_x_2 = data_table_2.Tx_m;
data_trans_y_2 = data_table_2.Ty_m;
data_trans_z_2 = data_table_2.Tz_m;
t2 = cat(2, data_trans_x_2, data_trans_y_2, data_trans_z_2);

%% error calculation
rot_err = calc_quaternion_mean_euclidean_diff(q1, q2);
[tx_err, ty_err, tz_err] = calc_abs_mean_translation_err(t1, t2);

%% save outputs
out_file = fopen(out_file, 'w');
fprintf(out_file, 'Quaternion_mean_euclidean_diff(rotation error) = %.2f\n', rot_err);
fprintf(out_file, "Abs_mean_Tx_err = %.2f\n", tx_err);
fprintf(out_file, "Abs_mean_Ty_err = %.2f\n", ty_err);
fprintf(out_file, "Abs_mean_Tz_err = %.2f\n", tz_err);
fclose(out_file);

%% % compare quaternions
function rot_err = calc_quaternion_mean_euclidean_diff(q1, q2)
    sub_q = q1-q2;
    add_q = q1+q2;
    norm_sub_q = vecnorm(sub_q, 2, 2);
    norm_add_q = vecnorm(add_q, 2, 2);
    q_err = min(norm_sub_q, norm_add_q);
    rot_err = mean(q_err);
end


%% % translation error
function [tx_err, ty_err, tz_err] = calc_abs_mean_translation_err(t1, t2)
    t_abs_sum = sum(abs(t1 - t2));
    samples_size = size(t1);
    t_err = t_abs_sum/samples_size(1);
    tx_err = t_err(1);
    ty_err = t_err(2);
    tz_err = t_err(3);
end


