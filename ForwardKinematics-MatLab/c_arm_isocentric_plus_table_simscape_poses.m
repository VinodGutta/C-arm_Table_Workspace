clc;
clear all;
close all;

%%
Sim_Time = 0.000001;
joint_values_simscape = [];
poses_simscape = [];
pose_count = 0;

%%
joint_values_path = "../Sample_data_files/random_joint_values.csv";
out_poses_path = "../Sample_data_files/c_arm_plus_table_simscape_out_poses.csv";
random_joint_values = readtable(joint_values_path);
random_joint_values_size = size(random_joint_values);
num_of_samples = random_joint_values_size(1);

%% joint values
c_arm_lateral = random_joint_values.c_arm_lateral_m;
c_arm_vertical = random_joint_values.c_arm_vertical_m;
c_arm_wigwag = random_joint_values.c_arm_wigwag_deg;
c_arm_horizontal = random_joint_values.c_arm_horizontal_m;
c_arm_tilt = random_joint_values.c_arm_tilt_deg;
c_arm_orbital = random_joint_values.c_arm_orbital_deg;

table_vertical = random_joint_values.table_vertical_m;
table_trend = random_joint_values.table_trend_deg;
table_tilt = random_joint_values.table_tilt_deg;
table_longitudinal = random_joint_values.table_longitudinal_m;
table_transverse = random_joint_values.table_transverse_m;

%%
for idx = 1:num_of_samples
    % count pose number
    pose_count = pose_count + 1;
    if rem(pose_count, 10) == 0
        disp(['pose count: ', num2str(pose_count)])
    end

    % get pose from simscape - sensor reading
    % joint input values
    v0 = c_arm_lateral(idx);
    v1 = c_arm_vertical(idx);
    v2 = c_arm_wigwag(idx);
    v3 = c_arm_horizontal(idx);
    v4 = c_arm_tilt(idx);
    v5 = c_arm_orbital(idx);

    v6 = table_vertical(idx);
    v7 = table_trend(idx);
    v8 = table_tilt(idx);
    v9 = table_longitudinal(idx);
    v10 = table_transverse(idx);
    
    % C-arm joint input values
    c_arm_lateral_joint_val = v0;
    c_arm_vertical_joint_val = v1;
    c_arm_wigwag_joint_val = v2;
    c_arm_horizontal_joint_val = v3;
    c_arm_tilt_joint_val = v4;
    c_arm_orbital_joint_val = v5;

    % table joint input values
    table_vertical_joint_val = v6;
    table_trend_joint_val = v7;
    table_tilt_joint_val = v8;
    table_longitudinal_joint_val = v9;
    table_transverse_joint_val = v10;

    % run simulation
    sim_out = sim("c_arm_isocentric_plus_table_model.slx");

    % get rotation
    rot_mat = sim_out.R.Data(:,:,1);
    q = rotm2quat(rot_mat); % quaternion [w x y z]
    
    % get translation
    t_x = sim_out.x.Data(1);
    t_y = sim_out.y.Data(1);
    t_z = sim_out.z.Data(1);

    % out data
    joint_values_simscape = [joint_values_simscape; v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10];
    poses_simscape = [poses_simscape; q, t_x, t_y, t_z];

end

%% save data
out_data_simscape = horzcat(joint_values_simscape, poses_simscape);
out_data_table_simscape = array2table(out_data_simscape);
out_data_table_simscape.Properties.VariableNames(1:18) = {'c_arm_lateral_m', ...
    'c_arm_vertical_m', 'c_arm_wigwag_deg', 'c_arm_horizontal_m', ...
    'c_arm_tilt_deg', 'c_arm_orbital_deg', 'table_vertical_m',...
    'table_trend_deg', 'table_tilt_deg', 'table_longitudinal_m',... 
    'table_transverse_m', 'quat_w', 'quat_x', 'quat_y', ...
    'quat_z', 'Tx_m', 'Ty_m', 'Tz_m'};
writetable(out_data_table_simscape, out_poses_path);