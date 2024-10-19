import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from transformation_mats import get_transf_mat_table_ee_to_c_arm_ee


if __name__ == "__main__":
    # input joints values
    input_joints_path = "./Sample_data_files/random_joint_values.csv"
    out_poses_path = "./Sample_data_files/c_arm_plus_table_dh_parameters_python_out_poses.csv"

    # read joint values
    joints_df = pd.read_csv(input_joints_path)
    joints_np = joints_df[['c_arm_lateral_m',
                           'c_arm_vertical_m', 
                           'c_arm_wigwag_deg', 
                           'c_arm_horizontal_m', 
                           'c_arm_tilt_deg', 
                           'c_arm_orbital_deg',
                           'table_vertical_m',
                           'table_trend_deg',
                           'table_tilt_deg',
                           'table_longitudinal_m',
                           'table_transverse_m']].to_numpy(dtype=np.float32)

    poses = []
    count = 0
    for joint_values in joints_np:
        count += 1
        c_arm_lat, c_arm_vert, c_arm_wigwag, c_arm_horz, c_arm_tilt, c_arm_orb, table_vert, table_trend, table_tilt, table_long, table_trans = joint_values
        pose_transf = get_transf_mat_table_ee_to_c_arm_ee(c_arm_lateral=c_arm_lat,
                                                          c_arm_vertical=c_arm_vert,
                                                          c_arm_wigwag=c_arm_wigwag,
                                                          c_arm_horizontal=c_arm_horz,
                                                          c_arm_tilt=c_arm_tilt,
                                                          c_arm_orbital=c_arm_orb,
                                                          table_vertical=table_vert,
                                                          table_trend=table_trend,
                                                          table_tilt=table_tilt,
                                                          table_longitudinal=table_long,
                                                          table_transverse=table_trans)
    
        pose_rot = R.from_matrix(pose_transf[0:3, 0:3])
        pose_quat = pose_rot.as_quat(canonical=True)  # x, y, z, w
        pose_translation = pose_transf[0:3, 3]

        x, y, z, w = pose_quat
        tx, ty, tz = pose_translation
        poses.append([w, x, y, z, tx, ty, tz])

        if count % 100 == 0:
            print(f"processed samples = {count}")

    # convert poses to data frame
    # header
    poses_col_names = ['quat_w', 'quat_x', 'quat_y', 'quat_z', 'Tx_m', 'Ty_m', 'Tz_m']
    poses_df = pd.DataFrame(poses, columns=poses_col_names)

    # concatenate joint values and poses dataframes
    out_df = pd.concat([joints_df, poses_df], axis=1)

    # output file
    out_df.to_csv(out_poses_path)

    print(f"Saved at {out_poses_path}")