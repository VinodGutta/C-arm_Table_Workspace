import sys
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from transformation_mats import get_transf_mat_table_ee_to_c_arm_ee
from transformation_mats import calc_transf_mat_c_arm_base_to_ee, calc_transf_mat_table_base_to_ee
from position_plot import save_position_plot
from workspace_sampling import sample_workspace
import time
import vedo
import os


global C_ARM_LAT
global C_ARM_VERT
global C_ARM_WIGWAG
global C_ARM_HORZ
global C_ARM_TILT
global C_ARM_ORB

global TABLE_VERT
global TABLE_TREND
global TABLE_TILT
global TABLE_LONG
global TABLE_TRANS

global C_ARM_LAT_RANGE
global C_ARM_VERT_RANGE
global C_ARM_WIGWAG_RANGE
global C_ARM_HORZ_RANGE
global C_ARM_TILT_RANGE
global C_ARM_ORB_RANGE

global TABLE_VERT_RANGE
global TABLE_TREND_RANGE
global TABLE_TILT_RANGE
global TABLE_LONG_RANGE
global TABLE_TRANS_RANGE


def save_metadata(out_txt_file, total_count, collision_free_count, num_of_samples):
    out_f = open(out_txt_file, 'w')
    out_f.write(f"NUM_OF_SAMPLES = {num_of_samples}\n")
    out_f.write(f"COLLISION_FREE = {COLLISION_FREE}\n\n")

    out_f.write(f"C_ARM_LAT = {C_ARM_LAT}\n")
    out_f.write(f"C_ARM_VERT = {C_ARM_VERT}\n")
    out_f.write(f"C_ARM_WIGWAG = {C_ARM_WIGWAG}\n")
    out_f.write(f"C_ARM_HORZ = {C_ARM_HORZ}\n")
    out_f.write(f"C_ARM_TILT = {C_ARM_TILT}\n")
    out_f.write(f"C_ARM_ORB = {C_ARM_ORB}\n")
    out_f.write(f"TABLE_VERT = {TABLE_VERT}\n")
    out_f.write(f"TABLE_TREND = {TABLE_TREND}\n")
    out_f.write(f"TABLE_TILT = {TABLE_TILT}\n")
    out_f.write(f"TABLE_LONG = {TABLE_LONG}\n")
    out_f.write(f"TABLE_TRANS = {TABLE_TRANS}\n\n")

    out_f.write(f"C_ARM_LAT_RANGE = {C_ARM_LAT_RANGE}\n")
    out_f.write(f"C_ARM_VERT_RANGE = {C_ARM_VERT_RANGE}\n")
    out_f.write(f"C_ARM_WIGWAG_RANGE = {C_ARM_WIGWAG_RANGE}\n")
    out_f.write(f"C_ARM_HORZ_RANGE = {C_ARM_HORZ_RANGE}\n")
    out_f.write(f"C_ARM_TILT_RANGE = {C_ARM_TILT_RANGE}\n")
    out_f.write(f"C_ARM_ORB_RANGE = {C_ARM_ORB_RANGE}\n")
    out_f.write(f"TABLE_VERT_RANGE = {TABLE_VERT_RANGE}\n")
    out_f.write(f"TABLE_TREND_RANGE = {TABLE_TREND_RANGE}\n")
    out_f.write(f"TABLE_TILT_RANGE = {TABLE_TILT_RANGE}\n")
    out_f.write(f"TABLE_LONG_RANGE = {TABLE_LONG_RANGE}\n")
    out_f.write(f"TABLE_TRANS_RANGE = {TABLE_TRANS_RANGE}\n\n")

    # print(f"total_count= {total_count}, collision_free_count = {collision_free_count}")
    out_f.write(f"out_num_of_samples={num_of_samples}\n")
    out_f.write(f"total samples = {total_count}\n")
    out_f.write(f"collision_free samples = {collision_free_count}\n")
    out_f.write(f"percentage of collision_free samples = {(collision_free_count / total_count) * 100}\n\n")
    out_f.close()


def get_random_joint_values():
    out_joints = {'c_arm_lat': 0.0, 'c_arm_vert': 0.0, 'c_arm_wigwag': 0.0, 'c_arm_horz': 0.0, 'c_arm_tilt': 0.0, 'c_arm_orb': 0.0,
                  'table_vert': 0.0, 'table_trend': 0.0, 'table_tilt': 0.0, 'table_long': 0.0, 'table_trans': 0.0}

    if C_ARM_LAT:
        out_joints['c_arm_lat'] = round((np.random.randint(low=C_ARM_LAT_RANGE[0]*100, high=C_ARM_LAT_RANGE[1]*100 + 1))/100, 2)
    if C_ARM_VERT:
        out_joints['c_arm_vert'] = round((np.random.randint(low=C_ARM_VERT_RANGE[0]*100, high=C_ARM_VERT_RANGE[1]*100 + 1))/100, 2)
    if C_ARM_WIGWAG:
        out_joints['c_arm_wigwag'] = float(np.random.randint(low=C_ARM_WIGWAG_RANGE[0],
                                                             high=int(C_ARM_WIGWAG_RANGE[1] + 1)))
    if C_ARM_HORZ:
        out_joints['c_arm_horz'] = round((np.random.randint(low=C_ARM_HORZ_RANGE[0]*100, high=C_ARM_HORZ_RANGE[1]*100 + 1))/100, 2)
    if C_ARM_TILT:
        out_joints['c_arm_tilt'] = float(np.random.randint(low=C_ARM_TILT_RANGE[0], high=int(C_ARM_TILT_RANGE[1] + 1)))
    if C_ARM_ORB:
        out_joints['c_arm_orb'] = float(np.random.randint(low=C_ARM_ORB_RANGE[0], high=int(C_ARM_ORB_RANGE[1] + 1)))

    if TABLE_VERT:
        out_joints['table_vert'] = round((np.random.randint(low=TABLE_VERT_RANGE[0]*100, high=TABLE_VERT_RANGE[1]*100 + 1))/100, 2)
    if TABLE_TREND:
        out_joints['table_trend'] = float(np.random.randint(low=TABLE_TREND_RANGE[0],
                                                            high=int(TABLE_TREND_RANGE[1] + 1)))
    if TABLE_TILT:
        out_joints['table_tilt'] = float(np.random.randint(low=TABLE_TILT_RANGE[0], high=int(TABLE_TILT_RANGE[1] + 1)))
    if TABLE_LONG:
        out_joints['table_long'] = round((np.random.randint(low=TABLE_LONG_RANGE[0]*100, high=TABLE_LONG_RANGE[1]*100 + 1))/100, 2)
    if TABLE_TRANS:
        out_joints['table_trans'] = round((np.random.randint(low=TABLE_TRANS_RANGE[0]*100, high=TABLE_TRANS_RANGE[1]*100 + 1))/100, 2)

    return out_joints


def check_end_pose_collision(c_arm_lateral, c_arm_vertical, c_arm_wigwag, c_arm_horizontal, c_arm_tilt, c_arm_orbital,
                             table_vertical, table_trend, table_tilt, table_longitudinal, table_transverse,
                             transf_c_arm_base_to_table_base, transf_c_arm_base_to_table_wheels_base, c_arm_pc,
                             table_top_mesh, table_body_mesh, table_wheels_base_mesh):
    # c-arm pose
    c_arm_pose = calc_transf_mat_c_arm_base_to_ee(c_arm_lateral, c_arm_vertical, c_arm_wigwag,
                                                  c_arm_horizontal, c_arm_tilt, c_arm_orbital)

    # table-top pose
    transf_table_base_to_ee = calc_transf_mat_table_base_to_ee(table_vertical, table_trend, table_tilt,
                                                               table_longitudinal, table_transverse)
    table_top_pose = transf_c_arm_base_to_table_base @ transf_table_base_to_ee

    # table body pose
    transf_c_arm_base_to_table_body = np.eye(4)
    transf_c_arm_base_to_table_body[0, 3] = 0.4
    transf_c_arm_base_to_table_body[1, 3] = 1.575
    transf_c_arm_base_to_table_body[2, 3] = table_vertical

    # transform c-arm point cloud
    c_arm_pc_cpy = c_arm_pc.clone()
    c_arm_pc_cpy.apply_transform(T=c_arm_pose, reset=False, concatenate=False)  # transform c-arm

    # transform table-top mesh
    table_top_mesh_cpy = table_top_mesh.clone()
    table_top_mesh_cpy.apply_transform(T=table_top_pose, reset=False, concatenate=False)

    # transform table body mesh
    table_body_mesh_cpy = table_body_mesh.clone()
    table_body_mesh_cpy.apply_transform(T=transf_c_arm_base_to_table_body, reset=False, concatenate=False)

    # transform table wheels base mesh
    table_wheels_base_mesh_cpy = table_wheels_base_mesh.clone()
    table_wheels_base_mesh_cpy.apply_transform(T=transf_c_arm_base_to_table_wheels_base, reset=False, concatenate=False)

    # collision points
    table_top_collision_pcd = table_top_mesh_cpy.inside_points(c_arm_pc_cpy.points())
    table_body_collision_pcd = table_body_mesh_cpy.inside_points(c_arm_pc_cpy.points())
    table_base_collision_pcd = table_wheels_base_mesh_cpy.inside_points(c_arm_pc_cpy.points())

    table_top_collision_pts_count = table_top_collision_pcd.points().shape[0]
    table_body_collision_pts_count = table_body_collision_pcd.points().shape[0]
    table_base_collision_pts_count = table_base_collision_pcd.points().shape[0]

    end_pose_collision = 0
    if table_top_collision_pts_count > 0 or table_body_collision_pts_count > 0 or table_base_collision_pts_count > 0:
        # print("Collision detected")
        end_pose_collision = 1
    # else:
        # print("No collision detected")

    # # visualize
    # plt = vedo.Plotter()
    # plt.show(table_top_mesh_cpy, table_body_mesh_cpy, table_wheels_base_mesh_cpy, c_arm_pc_cpy, axes=1).close()
    # sys.exit()

    return end_pose_collision


def generate_monte_carlo_poses(num_of_samples=100000, out_dir="./out", out_file="./out.csv", collision_free=False,
                               c_arm_lat_range=(-0.5, 0.5),
                               c_arm_vert_range=(0, 0.46),
                               c_arm_wigwag_range=(-10, 10),
                               c_arm_horz_range=(0, 0.15),
                               c_arm_tilt_range=(0, 0),
                               c_arm_orb_range=(0, 0),
                               table_vert_range=(0, 0.36),
                               table_trend_range=(-30, 30),
                               table_tilt_range=(-20, 20),
                               table_long_range=(0, 0.7),
                               table_trans_range=(-0.13, 0.13)):
    global C_ARM_LAT_RANGE
    global C_ARM_VERT_RANGE
    global C_ARM_WIGWAG_RANGE
    global C_ARM_HORZ_RANGE
    global C_ARM_TILT_RANGE
    global C_ARM_ORB_RANGE

    global TABLE_VERT_RANGE
    global TABLE_TREND_RANGE
    global TABLE_TILT_RANGE
    global TABLE_LONG_RANGE
    global TABLE_TRANS_RANGE

    # joint limits
    C_ARM_LAT_RANGE = c_arm_lat_range
    C_ARM_VERT_RANGE = c_arm_vert_range
    C_ARM_WIGWAG_RANGE = c_arm_wigwag_range
    C_ARM_HORZ_RANGE = c_arm_horz_range
    C_ARM_TILT_RANGE = c_arm_tilt_range  # (-90, 270)  # original range
    C_ARM_ORB_RANGE = c_arm_orb_range  # (-100, 100)  # original range

    TABLE_VERT_RANGE = table_vert_range
    TABLE_TREND_RANGE = table_trend_range
    TABLE_TILT_RANGE = table_tilt_range
    TABLE_LONG_RANGE = table_long_range
    TABLE_TRANS_RANGE = table_trans_range

    total_count = 0
    collision_free_count = 0
    cut_off_count = 0
    jnts_arr = []
    poses_arr = []
    end_pose_collide_arr = []

    # collision detection
    # 3d point clouds and meshes path
    table_top_mesh_path = "./3d_inputs/table_top_watertight_mesh.ply"
    table_wheels_base_mesh_path = "./3d_inputs/table_wheels_base_watertight_mesh.ply"
    table_body_mesh_path = "./3d_inputs/table_body_sphere_watertight_mesh.ply"
    c_arm_pcd_pts_path = "./3d_inputs/c_arm_pcd_pts.npy"

    # read point clouds and meshes
    c_arm_pts = np.load(c_arm_pcd_pts_path, allow_pickle=True)
    c_arm_pc = vedo.Points(inputobj=c_arm_pts, r=10, c=(0, 0, 1))
    table_top_mesh = vedo.file_io.load(table_top_mesh_path)
    table_body_mesh = vedo.file_io.load(table_body_mesh_path)
    table_wheels_base_mesh = vedo.file_io.load(table_wheels_base_mesh_path)

    # color
    table_body_mesh.c(color=(1, 0, 0), alpha=1)
    table_top_mesh.c(color=(1, 0, 0), alpha=1)
    table_wheels_base_mesh.c(color=(1, 0, 0), alpha=1)

    # initial pose of c-arm base with table base
    transf_c_arm_base_to_table_base = np.eye(4)
    transf_c_arm_base_to_table_base[0, 3] = 0.4
    transf_c_arm_base_to_table_base[1, 3] = 1.575

    # table wheels base pose
    transf_c_arm_base_to_table_wheels_base = np.eye(4)
    transf_c_arm_base_to_table_wheels_base[0, 3] = 0.4 - 0.15
    transf_c_arm_base_to_table_wheels_base[1, 3] = 1.575

    start_t = time.time()
    while cut_off_count < num_of_samples:
        # joint values
        jnts = get_random_joint_values()
        v0 = jnts['c_arm_lat']
        v1 = jnts['c_arm_vert']
        v2 = jnts['c_arm_wigwag']
        v3 = jnts['c_arm_horz']
        v4 = jnts['c_arm_tilt']
        v5 = jnts['c_arm_orb']
        v6 = jnts['table_vert']
        v7 = jnts['table_trend']
        v8 = jnts['table_tilt']
        v9 = jnts['table_long']
        v10 = jnts['table_trans']

        # pose values
        pose_transf = get_transf_mat_table_ee_to_c_arm_ee(v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10)
        pose_rot = R.from_matrix(pose_transf[0:3, 0:3])
        pose_quat = pose_rot.as_quat(canonical=True)  # x, y, z, w
        pose_translation = pose_transf[0:3, 3]

        x, y, z, w = pose_quat
        tx, ty, tz = pose_translation

        end_pose_collide = check_end_pose_collision(c_arm_lateral=v0, c_arm_vertical=v1, c_arm_wigwag=v2,
                                                    c_arm_horizontal=v3, c_arm_tilt=v4, c_arm_orbital=v5,
                                                    table_vertical=v6, table_trend=v7, table_tilt=v8,
                                                    table_longitudinal=v9, table_transverse=v10,
                                                    transf_c_arm_base_to_table_base=transf_c_arm_base_to_table_base,
                                                    transf_c_arm_base_to_table_wheels_base=transf_c_arm_base_to_table_wheels_base,
                                                    c_arm_pc=c_arm_pc, table_top_mesh=table_top_mesh,
                                                    table_body_mesh=table_body_mesh,
                                                    table_wheels_base_mesh=table_wheels_base_mesh)

        if collision_free:
            if end_pose_collide == 0:
                jnts_arr.append([v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10])
                poses_arr.append([w, x, y, z, tx, ty, tz])
                end_pose_collide_arr.append(end_pose_collide)
                cut_off_count += 1
                collision_free_count += 1
        else:
            jnts_arr.append([v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10])
            poses_arr.append([w, x, y, z, tx, ty, tz])
            end_pose_collide_arr.append(end_pose_collide)
            cut_off_count += 1
            if end_pose_collide == 0:
                collision_free_count += 1

        total_count += 1
        if total_count % 5000 == 0:
            time_elapsed = time.time() - start_t
            print(f"total count = {total_count}; finished samples = {cut_off_count}/{num_of_samples};"
                  f" collision_free_samples = {collision_free_count};"
                  f" time_elapsed = {round(time_elapsed/60, 2)} min")

    # joint df
    jnts_col_names = ['c-arm lateral (m)', 'c-arm vertical (m)', 'c-arm wigwag (degrees)', 'c-arm horizontal (m)',
                      'c-arm tilt (degrees)', 'c-arm orbital (degrees)', 'table vertical (m)', 'table trend (degrees)',
                      'table tilt (degrees)', 'table longitudinal (m)', 'table transverse (m)']
    jnts_df = pd.DataFrame(jnts_arr, columns=jnts_col_names)

    # poses df
    poses_col_names = ['quat_w', 'quat_x', 'quat_y', 'quat_z', 'Tx (m)', 'Ty (m)', 'Tz (m)']
    poses_df = pd.DataFrame(poses_arr, columns=poses_col_names)

    # end pose collision df
    end_pose_collision_col_names = ['end pose collision']
    end_pose_collision_df = pd.DataFrame(end_pose_collide_arr, columns=end_pose_collision_col_names)

    # combine
    out_csv_file = os.path.join(out_dir, out_file)
    out_df = pd.concat([jnts_df, poses_df, end_pose_collision_df], axis=1)
    out_df.to_csv(out_csv_file)

    # save metadata
    out_txt_file = os.path.join(out_dir, out_csv_file.split("/")[-1].split(".")[0] + "_metadata.txt")
    save_metadata(out_txt_file, total_count, collision_free_count, num_of_samples)

    # save position plot
    save_position_plot(out_csv_file, out_dir)


def joints_extreme_monte_carlo_poses(num_of_samples, out_dir, c_arm_tilt_range, c_arm_orb_range):
    if C_ARM_LAT:
        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_c_arm_lat_left_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   c_arm_lat_range=(-0.5, -0.5),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_c_arm_lat_right_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   c_arm_lat_range=(0.5, 0.5),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

    if C_ARM_VERT:
        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_c_arm_vert_left_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   c_arm_vert_range=(0.0, 0.0),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_c_arm_vert_right_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   c_arm_vert_range=(0.46, 0.46),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

    if C_ARM_WIGWAG:
        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_c_arm_wigwag_left_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   c_arm_wigwag_range=(-10.0, -10.0),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_c_arm_wigwag_right_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   c_arm_wigwag_range=(10.0, 10.0),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

    if C_ARM_HORZ:
        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_c_arm_horz_left_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   c_arm_horz_range=(0.0, 0.0),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_c_arm_horz_right_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   c_arm_horz_range=(0.15, 0.15),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

    if TABLE_VERT:
        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_table_vert_left_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   table_vert_range=(0.0, 0.0),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_table_vert_right_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   table_vert_range=(0.36, 0.36),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

    if TABLE_LONG:
        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_table_long_left_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   table_long_range=(0.0, 0.0),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_table_long_right_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   table_long_range=(0.7, 0.7),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

    if TABLE_TRANS:
        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_table_trans_left_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   table_trans_range=(-0.13, -0.13),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)

        generate_monte_carlo_poses(num_of_samples=num_of_samples, out_dir=out_dir,
                                   out_file=f"c_arm_table_table_trans_right_extreme_random_{num_of_samples}.csv",
                                   collision_free=COLLISION_FREE,
                                   table_trans_range=(0.13, 0.13),
                                   c_arm_tilt_range=c_arm_tilt_range,
                                   c_arm_orb_range=c_arm_orb_range)


def combine_csv_files(in_dir, out_file_name):
    csv_files = []
    for f in os.listdir(in_dir):
        if f.endswith(".csv"):
            csv_files.append(os.path.join(in_dir, f))

    df_lst = []
    for in_csv in csv_files:
        # print(in_csv)
        csv_df = pd.read_csv(in_csv)
        df_lst.append(csv_df)

    combined_df = pd.concat(df_lst)
    combined_df.to_csv(os.path.join(in_dir, out_file_name))
    print(f"csv combined_shape = {combined_df.shape}")


if __name__ == "__main__":
    # to vary dofs
    vary_dofs = ["5_dofs", "6_dofs", "6_plus_1_dof_trans", "6_plus_2_dofs_trans_vert", "6_plus_3_dofs", "table_3dofs"]
    c_arm_joints_enable = [[False, True, True, True, True, True],
                           [True, True, True, True, True, True],
                           [True, True, True, True, True, True],
                           [True, True, True, True, True, True],
                           [True, True, True, True, True, True],
                           [False, False, False, False, True, True]]
    table_joints_enable = [[False, False, False, False, False],
                           [False, False, False, False, False],
                           [False, False, False, False, True],
                           [True, False, False, False, True],
                           [True, False, False, True, True],
                           [True, False, False, True, True]]

    # poses_types
    pose_types = ["PA", "AP", "V1", "V2", "Ver", "Lat"]

    c_arm_tilt_pose_types = [0.0, 180.0, -30.0, 25.0, 0.0, 0.0]
    c_arm_orbital_pose_types = [0.0, 0.0, -30.0, 45.0, -35.0, -90.0]

    # multiple runs with varying dofs and pose types
    t1 = time.time()
    for idx1, v_dofs in enumerate(vary_dofs):
        if v_dofs is None:
            continue
        for idx2, p_type in enumerate(pose_types):
            t2 = time.time()
            print(f"Generating...{v_dofs}...{p_type}; time_elapsed_min={round((t2-t1)/60, 3)}")

            C_ARM_LAT = c_arm_joints_enable[idx1][0]
            C_ARM_VERT = c_arm_joints_enable[idx1][1]
            C_ARM_WIGWAG = c_arm_joints_enable[idx1][2]
            C_ARM_HORZ = c_arm_joints_enable[idx1][3]
            C_ARM_TILT = c_arm_joints_enable[idx1][4]
            C_ARM_ORB = c_arm_joints_enable[idx1][5]

            TABLE_VERT = table_joints_enable[idx1][0]
            TABLE_TREND = table_joints_enable[idx1][1]
            TABLE_TILT = table_joints_enable[idx1][2]
            TABLE_LONG = table_joints_enable[idx1][3]
            TABLE_TRANS = table_joints_enable[idx1][4]

            COLLISION_FREE = False  # enable this to get only collision free poses
            out_dir = f"./out/{p_type}/{v_dofs}"
            if not os.path.isdir(out_dir):
                os.makedirs(out_dir)
            else:
                print("Out directory already exists. Please choose a different directory")
                sys.exit()

            # random samples
            NUM_OF_SAMPLES = 100000
            out_random_samples_csv_file = f"c_arm_table_random_{NUM_OF_SAMPLES}.csv"
            generate_monte_carlo_poses(num_of_samples=NUM_OF_SAMPLES, out_dir=out_dir,
                                       out_file=out_random_samples_csv_file, collision_free=COLLISION_FREE,
                                       c_arm_tilt_range=(c_arm_tilt_pose_types[idx2], c_arm_tilt_pose_types[idx2]),
                                       c_arm_orb_range=(c_arm_orbital_pose_types[idx2], c_arm_orbital_pose_types[idx2]))

            NUM_OF_SAMPLES = 10000
            joints_extreme_monte_carlo_poses(num_of_samples=NUM_OF_SAMPLES, out_dir=out_dir,
                                             c_arm_tilt_range=(c_arm_tilt_pose_types[idx2],
                                                               c_arm_tilt_pose_types[idx2]),
                                             c_arm_orb_range=(c_arm_orbital_pose_types[idx2],
                                                              c_arm_orbital_pose_types[idx2]))

            # combine initial random samples with extreme random samples
            combine_csv_files(out_dir, "c_arm_table_combined.csv")

            # combined position plot
            combined_position_plot_in_csv_file = os.path.join(out_dir, "c_arm_table_combined.csv")
            save_position_plot(combined_position_plot_in_csv_file, out_dir)

            # workspace sampling
            sample_workspace(combined_position_plot_in_csv_file, res_cm=5.0)
    print(f"total_time = {round((time.time() - t1) / 60, 2)} mins")
