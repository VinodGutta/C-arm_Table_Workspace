import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
from scipy.linalg import norm
from scipy.optimize import minimize
from transformation_mats import get_transf_mat_table_ee_to_c_arm_ee, \
    calc_transf_mat_c_arm_base_to_ee, calc_transf_mat_table_base_to_ee
import vedo
import time
import argparse
import os
import ast
import sys
import multiprocessing as mp
import shutil


# Rotation abs diff
def axis_rot_error(r1, r2):
    return min(abs(r1-r2), 360 - abs(r1-r2))


# IK objective minimization function
def ik_objective_fn(x, opt_jnts, target_pos: np.ndarray, target_rot_zyx: np.ndarray, c_arm_tilt: float, c_arm_orb: float):
    # Initial pose settings
    init_pose_jnts = {'c_arm_lat': 0,
                      'c_arm_vert': 0, 
                      'c_arm_wigwag': 0, 
                      'c_arm_horz': 0.075,
                      'c_arm_tilt': 0,
                      'c_arm_orb': 0,
                      'table_vert': 0, 
                      'table_trend': 0, 
                      'table_tilt': 0, 
                      'table_long': 0.35, 
                      'table_trans': 0}
    
    # jnts
    jnts = init_pose_jnts.copy()
    jnts['c_arm_tilt'] = c_arm_tilt
    jnts['c_arm_orb'] = c_arm_orb
    for idx, jnt in enumerate(opt_jnts):
        jnts[jnt] = x[idx]

    out_pose = get_transf_mat_table_ee_to_c_arm_ee(c_arm_lateral=jnts['c_arm_lat'],
                                                   c_arm_vertical=jnts['c_arm_vert'],
                                                   c_arm_wigwag=jnts['c_arm_wigwag'],
                                                   c_arm_horizontal=jnts['c_arm_horz'],
                                                   c_arm_tilt=jnts['c_arm_tilt'],
                                                   c_arm_orbital=jnts['c_arm_orb'],
                                                   table_vertical=jnts['table_vert'],
                                                   table_trend=jnts['table_trend'],
                                                   table_tilt=jnts['table_tilt'],
                                                   table_longitudinal=jnts['table_long'],
                                                   table_transverse=jnts['table_trans'])
    
    out_Rmat = R.from_matrix(out_pose[0:3, 0:3])
    out_R_zyx = out_Rmat.as_euler(seq="zyx", degrees=True)
    out_pos = np.array([out_pose[0, 3], out_pose[1, 3], out_pose[2, 3]])

    # error to minimize
    pos_err = norm(abs(target_pos - out_pos), ord=2)
    rot_abs_diff = [axis_rot_error(target_rot_zyx[0], out_R_zyx[0]),
                    axis_rot_error(target_rot_zyx[1], out_R_zyx[1]),
                    axis_rot_error(target_rot_zyx[2], out_R_zyx[2])]
    rot_err = norm(rot_abs_diff, ord=2)
    err = (pos_err**2) + (rot_err**2)
    
    return err


# check for collision
def check_end_pose_collision(c_arm_lateral, c_arm_vertical, c_arm_wigwag, c_arm_horizontal, c_arm_tilt, c_arm_orbital,
                             table_vertical, table_trend, table_tilt, table_longitudinal, table_transverse,
                             c_arm_pc, table_top_mesh, table_body_mesh, table_wheels_base_mesh):
    # initial pose of c-arm base with table base
    transf_c_arm_base_to_table_base = np.eye(4)
    transf_c_arm_base_to_table_base[0, 3] = 0.4
    transf_c_arm_base_to_table_base[1, 3] = 1.575

    # table wheels base pose
    transf_c_arm_base_to_table_wheels_base = np.eye(4)
    transf_c_arm_base_to_table_wheels_base[0, 3] = 0.4 - 0.15
    transf_c_arm_base_to_table_wheels_base[1, 3] = 1.575
    
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


# random joint values
def get_random_joint_values(jnts, jnts_bounds):
    out = []
    for idx, jnt in enumerate(jnts):
        if jnt == 'c_arm_wigwag':
            out.append(float(np.random.randint(low=jnts_bounds[idx][0], high=int(jnts_bounds[idx][1] + 1))))
        else:
            out.append(round((np.random.randint(low=jnts_bounds[idx][0]*1000, high=jnts_bounds[idx][1]*1000 + 1))/1000, 3))

    return out


# quaternion to Euler
def convert_quat2euler(qx, qy, qz, qw):
    temp_R = R.from_quat([qx, qy, qz, qw])
    out = temp_R.as_euler(seq="zyx", degrees=True)
    return out


# inverse kinematics
def inverse_kinematics(process_idx, opt_jnts, opt_jnts_bnds, c_arm_tilt, c_arm_orb, target_pos_np, target_rot_zyx, temp_dir):
    print(f"From Process={process_idx}; num_of_poses={target_pos_np.shape[0]}; Started this thread.", flush=True)

    # for reproducibility
    np.random.seed(100)

    # max restarts
    max_rnd_cfgs = 100
    max_iterations = 1000
    tol = 1e-9

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

    # Initial pose: Joint values
    init_pose_jnts = {'c_arm_lat': 0,
                     'c_arm_vert': 0, 
                     'c_arm_wigwag': 0, 
                     'c_arm_horz': 0.075,
                     'c_arm_tilt': 0,
                     'c_arm_orb': 0,
                     'table_vert': 0, 
                     'table_trend': 0, 
                     'table_tilt': 0, 
                     'table_long': 0.35, 
                     'table_trans': 0}


    # inverse kinematics
    iterations = []
    num_of_rnd_cfgs = []
    collision = []
    ik_success = []
    ik_message = []
    ik_jnts = []
    ik_pose = []

    # iterate through samples
    poses_count = 0
    total_poses = target_pos_np.shape[0]
    start_t = time.time()
    for pos, rot in zip(target_pos_np, target_rot_zyx):
        poses_count += 1
        for rnd_cfg_count in range(1, max_rnd_cfgs+1):
            # inverse kinematics - start with random joint values
            rnd_init_jnts = get_random_joint_values(jnts=opt_jnts, jnts_bounds=opt_jnts_bnds)
            ik_res = minimize(fun=ik_objective_fn, x0=rnd_init_jnts, 
                              args=(opt_jnts, pos, rot, c_arm_tilt, c_arm_orb),
                              method="L-BFGS-B", bounds=opt_jnts_bnds,
                              tol=tol, options={'maxiter': max_iterations})
            
            # jnts
            ik_res_jnts = init_pose_jnts.copy()
            ik_res_jnts['c_arm_tilt'] = c_arm_tilt
            ik_res_jnts['c_arm_orb'] = c_arm_orb
            for idx, jnt in enumerate(opt_jnts):
                ik_res_jnts[jnt] = round(ik_res.x[idx], 5)
            ik_res_jnts['c_arm_wigwag'] = round(ik_res_jnts['c_arm_wigwag'], 2)
            
            # collision check
            collision_deteceted = check_end_pose_collision(c_arm_lateral=ik_res_jnts['c_arm_lat'],
                                                           c_arm_vertical=ik_res_jnts['c_arm_vert'],
                                                           c_arm_wigwag=ik_res_jnts['c_arm_wigwag'],
                                                           c_arm_horizontal=ik_res_jnts['c_arm_horz'],
                                                           c_arm_tilt=ik_res_jnts['c_arm_tilt'],
                                                           c_arm_orbital=ik_res_jnts['c_arm_orb'],
                                                           table_vertical=ik_res_jnts['table_vert'],
                                                           table_trend=ik_res_jnts['table_trend'],
                                                           table_tilt=ik_res_jnts['table_tilt'],
                                                           table_longitudinal=ik_res_jnts['table_long'],
                                                           table_transverse=ik_res_jnts['table_trans'],
                                                           c_arm_pc=c_arm_pc, 
                                                           table_top_mesh=table_top_mesh,
                                                           table_body_mesh=table_body_mesh,
                                                           table_wheels_base_mesh=table_wheels_base_mesh)

            if ik_res.success and (not collision_deteceted):
                # print(f"ik_res_jnts={ik_res_jnts}\n target_pos={pos}\n target_rot={rot}\n\n")
                break

        # IK results
        ik_res_pose = get_transf_mat_table_ee_to_c_arm_ee(c_arm_lateral=ik_res_jnts['c_arm_lat'],
                                                          c_arm_vertical=ik_res_jnts['c_arm_vert'],
                                                          c_arm_wigwag=ik_res_jnts['c_arm_wigwag'],
                                                          c_arm_horizontal=ik_res_jnts['c_arm_horz'],
                                                          c_arm_tilt=ik_res_jnts['c_arm_tilt'],
                                                          c_arm_orbital=ik_res_jnts['c_arm_orb'],
                                                          table_vertical=ik_res_jnts['table_vert'],
                                                          table_trend=ik_res_jnts['table_trend'],
                                                          table_tilt=ik_res_jnts['table_tilt'],
                                                          table_longitudinal=ik_res_jnts['table_long'],
                                                          table_transverse=ik_res_jnts['table_trans'])
        
        ik_jnts.append([ik_res_jnts['c_arm_lat'], ik_res_jnts['c_arm_vert'], ik_res_jnts['c_arm_wigwag'], 
                        ik_res_jnts['c_arm_horz'], ik_res_jnts['c_arm_tilt'], ik_res_jnts['c_arm_orb'], 
                        ik_res_jnts['table_vert'], ik_res_jnts['table_trend'], ik_res_jnts['table_tilt'], 
                        ik_res_jnts['table_long'], ik_res_jnts['table_trans']])
        
        ik_res_quat = R.from_matrix(ik_res_pose[0:3, 0:3]).as_quat() # x y z w
        ik_pose.append([ik_res_quat[3], ik_res_quat[0], ik_res_quat[1], ik_res_quat[2], 
                        ik_res_pose[0, 3], ik_res_pose[1, 3], ik_res_pose[2, 3]]) # qw, qx, qy, qz, tx, ty, tz
        ik_message.append(ik_res.message)
        ik_success.append(ik_res.success)
        collision.append(collision_deteceted)
        num_of_rnd_cfgs.append(rnd_cfg_count)
        iterations.append(ik_res.nit)

        if poses_count % 100 == 0:
            time_elapsed = time.time() - start_t
            print(f"Process={process_idx}; Processed: {poses_count}/{total_poses} poses; time elapsed = {time_elapsed:.2f} s", flush=True)

    out_data = {'ik_jnts': ik_jnts,
                'ik_pose': ik_pose, 
                'collision': collision, 
                'iterations': iterations,
                'num_of_rnd_cfgs': num_of_rnd_cfgs,
                'ik_success': ik_success,
                'ik_message': ik_message}
    
    # save data
    out_path = os.path.join(temp_dir, f"data_{process_idx}.npy")
    np.save(out_path, out_data, allow_pickle=True)


def get_start_end_idxs(input_num_of_poses, num_of_mp_threads):
    # sample size --> no. of samples per thread
    if input_num_of_poses%num_of_mp_threads == 0:
        sample_size = int(input_num_of_poses/num_of_mp_threads)
    elif input_num_of_poses%(num_of_mp_threads-1) == 0:
        sample_size = int(input_num_of_poses/(num_of_mp_threads-1)) - 1
    else:
        sample_size = int(input_num_of_poses/(num_of_mp_threads-1))

    # out indexes
    out_start_idx = []
    out_end_idx = []
    prev_start_idx = 0
    for _ in range(num_of_mp_threads):
        out_start_idx.append(prev_start_idx)
        curr_end_idx = prev_start_idx + sample_size
        
        if curr_end_idx <= input_num_of_poses:
            end_idx = curr_end_idx
        else:
            end_idx = input_num_of_poses

        out_end_idx.append(end_idx)
        prev_start_idx = end_idx

    return out_start_idx, out_end_idx


if __name__=="__main__":
    start_t = time.time()
    
    num_of_mp_threads = 4
    out_dir = "ik_out"

    # temp directory to save each process data
    temp_dir = os.path.join(out_dir, "temp")
    os.mkdir(temp_dir)

    # surgical configuration
    # PA: C-arm tilt = 0; orbital = 0
    c_arm_tilt = 0
    c_arm_orb = 0

    # input csv path
    input_csv_path = "target_poses.csv"
    output_csv_path = os.path.join(out_dir, f"ik_output.csv")
    
    # optimize joints
    opt_jnts = ['c_arm_lat', 'c_arm_vert', 'c_arm_wigwag', 'c_arm_horz', 'table_vert', 'table_long', 'table_trans']

    # Bounds
    jnts_bnds = {'c_arm_lat': (-0.5, 0.5),
                 'c_arm_vert': (0, 0.46), 
                 'c_arm_wigwag': (-10, 10), 
                 'c_arm_horz': (0, 0.15),
                 'table_vert': (0, 0.36), 
                 'table_long': (0, 0.7), 
                 'table_trans': (-0.13, 0.13)}

    # optimize  joints bounds
    opt_jnts_bnds = []
    for jnt in opt_jnts:
        opt_jnts_bnds.append(jnts_bnds[jnt])

    print(f"opt_jnts={opt_jnts}", flush=True)
    print(f"opt_jnts_bnds={opt_jnts_bnds}", flush=True)

    # read poses
    target_df = pd.read_csv(input_csv_path)
    target_quats_np = target_df[['quat_w', 'quat_x', 'quat_y', 'quat_z']].to_numpy(dtype=np.float32)
    target_pos_np = target_df[['Tx (m)', 'Ty (m)', 'Tz (m)']].to_numpy(dtype=np.float32)
    target_rot_zyx = [convert_quat2euler(q[1], q[2], q[3], q[0]) for q in target_quats_np]

    # multiple parallel threads
    start_idxs, end_idxs = get_start_end_idxs(input_num_of_poses=target_pos_np.shape[0], num_of_mp_threads=num_of_mp_threads)
    mp_threads = []
    process_idx = 0
    for s_idx, e_idx in zip(start_idxs, end_idxs):
        mp_threads.append(mp.Process(target=inverse_kinematics, args=(process_idx, 
                                                                      opt_jnts, 
                                                                      opt_jnts_bnds,
                                                                      c_arm_tilt, 
                                                                      c_arm_orb,
                                                                      target_pos_np[s_idx: e_idx], 
                                                                      target_rot_zyx[s_idx: e_idx], 
                                                                      temp_dir)))
        print(f"Process={process_idx}; start_idx={s_idx}; end_idx={e_idx}", flush=True)
        process_idx += 1
    
    for p_idx in range(len(mp_threads)):
        print(f"starting process = {p_idx}", flush=True)
        mp_threads[p_idx].start()

    # join mp threads
    for idx, mp_thr in enumerate(mp_threads):
        if mp_thr.is_alive():
            mp_thr.join()
            print(f"Joined Process={idx}", flush=True)
        else:
            print(f"Process={idx} killed before joining", flush=True)


    # join all outputs
    # inverse kinematics
    iterations = []
    num_of_rnd_cfgs = []
    collision = []
    ik_success = []
    ik_message = []
    ik_jnts = []
    ik_pose = []
    for p_idx in range(len(mp_threads)):
        print(f"Process={p_idx}, reading data", flush=True)
        data_path = os.path.join(temp_dir, f"data_{p_idx}.npy")
        q_out = np.load(data_path, allow_pickle=True).item()
        iterations += q_out['iterations']
        num_of_rnd_cfgs += q_out['num_of_rnd_cfgs']
        collision += q_out['collision']
        ik_success += q_out['ik_success']
        ik_message += q_out['ik_message']
        ik_jnts += q_out['ik_jnts']
        ik_pose += q_out['ik_pose']


    # save output csv file
    df1_col_names = ['c-arm lateral (m)', 'c-arm vertical (m)', 'c-arm wigwag (degrees)', 'c-arm horizontal (m)',
                     'c-arm tilt (degrees)', 'c-arm orbital (degrees)', 'table vertical (m)', 
                     'table trend (degrees)', 'table tilt (degrees)', 'table longitudinal (m)', 'table transverse (m)']
    df1 = pd.DataFrame(ik_jnts, columns=df1_col_names)
    
    df2_col_names = ['quat_w', 'quat_x', 'quat_y', 'quat_z', 'Tx (m)', 'Ty (m)', 'Tz (m)']
    df2 = pd.DataFrame(ik_pose, columns=df2_col_names)

    df3 = pd.DataFrame({'collision detected': collision,
                        'num_of_iterations': iterations,
                        'num_of_rnd_start_jnts': num_of_rnd_cfgs,
                        'success': ik_success,
                        'message': ik_message})
    
    # concatenate joint values and poses dataframes
    out_df = pd.concat([df1, df2, df3], axis=1)

    # check dimensions
    if out_df.shape[0] == target_df.shape[0]:
        # output file
        out_df.to_csv(output_csv_path)
    else:
        print("ERROR: Input and output number of poses not matched. NOT SAVING THE OUTPUTS.")

    # remove temp dir
    shutil.rmtree(temp_dir)

    run_time = time.time() - start_t
    print(f"Saved at {output_csv_path}; Total runtime = {run_time/60:.2f} mins")
    