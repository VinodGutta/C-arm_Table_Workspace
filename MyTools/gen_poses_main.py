import sys
import numpy as np
import pandas as pd
import time
import os


# generate random initial and target boxes
def get_random_init_and_target_boxes(init_sampled_csv_path, target_sampled_csv_path, num_of_out_poses, max_iter_before_exit, min_dist_each_axis):
    print("Generating random initial and target boxes...")

    # read csv files as data frames
    init_sampled_df = pd.read_csv(init_sampled_csv_path, header=0, index_col=0)
    target_sampled_df = pd.read_csv(target_sampled_csv_path, header=0, index_col=0)

    # collision free boxes
    init_collision_free_boxes = init_sampled_df[(init_sampled_df["Collision"] == 0.0) |
                                                (init_sampled_df["Collision"] == 2.0)]

    target_collision_free_boxes = target_sampled_df[(target_sampled_df["Collision"] == 0.0) |
                                                    (target_sampled_df["Collision"] == 2.0)]
    
    init_collision_free_boxes_np = init_collision_free_boxes.to_numpy()
    target_collision_free_boxes_np = target_collision_free_boxes.to_numpy()
    
    # check if collision free boxes are present in the input files
    out_init_boxes = None
    out_target_boxes = None

    if (init_collision_free_boxes_np.shape[0] == 0) or (target_collision_free_boxes_np.shape[0] == 0):
        return out_init_boxes, out_target_boxes
    

    poses_count = 0
    pose_not_found_count = 0
    init_poses_idx_lst = []
    target_poses_idx_lst = []

    start_t = time.time()
    while True:
        rand_init_box_idx = np.random.randint(low=0, high=init_collision_free_boxes_np.shape[0])
        rand_target_box_idx = np.random.randint(low=0, high=target_collision_free_boxes_np.shape[0])

        # check if distance in each axis has minimum value
        x_dist = np.abs(init_collision_free_boxes_np[rand_init_box_idx][0] - target_collision_free_boxes_np[rand_target_box_idx][0])
        y_dist = np.abs(init_collision_free_boxes_np[rand_init_box_idx][1] - target_collision_free_boxes_np[rand_target_box_idx][1])
        z_dist = np.abs(init_collision_free_boxes_np[rand_init_box_idx][2] - target_collision_free_boxes_np[rand_target_box_idx][2])

        # minimum distance check
        if (x_dist < min_dist_each_axis) or (y_dist < min_dist_each_axis) or (z_dist < min_dist_each_axis):
            pose_not_found_count += 1

        # check whether this random pose boxes already exists and captured in the lists
        elif rand_init_box_idx in init_poses_idx_lst:
            exist_target_box_idx = target_poses_idx_lst[init_poses_idx_lst.index(rand_init_box_idx)]
            if exist_target_box_idx == rand_target_box_idx:
                # this initial and target pose was captured in before samples
                pose_not_found_count += 1
            else:
                # append to initial and target poses list
                init_poses_idx_lst.append(rand_init_box_idx)
                target_poses_idx_lst.append(rand_target_box_idx)
                poses_count += 1
                pose_not_found_count = 0

                if poses_count % 1000 == 0:
                    print(f"Num of pose boxes identified = {poses_count}; Time elapsed = {time.time()-start_t:.2f} s")
        else:
            # append to initial and target poses list
            init_poses_idx_lst.append(rand_init_box_idx)
            target_poses_idx_lst.append(rand_target_box_idx)
            poses_count += 1
            pose_not_found_count = 0

            if poses_count % 1000 == 0:
                    print(f"Num of pose boxes identified = {poses_count}; Time elapsed = {time.time()-start_t:.2f} s")
        
        if pose_not_found_count == max_iter_before_exit:
            print(f"Stopping... Can't find more poses. Tried {max_iter_before_exit} times.")
            break

        if poses_count == num_of_out_poses:
            print(f"Sampled {num_of_out_poses} random initial and target pose boxes. Stopping...")
            break

    # outputs
    out_init_boxes = init_collision_free_boxes_np[init_poses_idx_lst]
    out_target_boxes = target_collision_free_boxes_np[target_poses_idx_lst]

    return out_init_boxes, out_target_boxes



# get a random sample from a box
def get_rand_sample(in_df, bx, by, bz, res_cm):
    x1 = bx - (res_cm / 2)
    x2 = bx + (res_cm / 2)

    y1 = by - (res_cm / 2)
    y2 = by + (res_cm / 2)

    z1 = bz - (res_cm / 2)
    z2 = bz + (res_cm / 2)

    sample_poses = in_df[((in_df["Tx (m)"] >= (x1/100)) & (in_df["Tx (m)"] < (x2/100))) &
                         ((in_df["Ty (m)"] >= (y1/100)) & (in_df["Ty (m)"] < (y2/100))) &
                         ((in_df["Tz (m)"] >= (z1/100)) & (in_df["Tz (m)"] < (z2/100)))]

    while True:
        rand_idx = np.random.randint(0, sample_poses.shape[0])
        if sample_poses.iloc[rand_idx]["end pose collision"] == 0:
            break

    return sample_poses.iloc[rand_idx]


# generate initial and random poses
def get_rand_init_target_poses(initial_combined_csv_in_path, target_combined_csv_in_path, init_boxes, target_boxes, max_iter_pose_before_exit, min_dist_each_axis):
    print("Generating random initial and target poses...")

    # read csv files
    init_combined_df = pd.read_csv(initial_combined_csv_in_path, header=0, index_col=0)
    init_combined_df = init_combined_df.drop("Unnamed: 0", axis=1)
    cols = init_combined_df.columns

    target_combined_df = pd.read_csv(target_combined_csv_in_path, header=0, index_col=0)
    target_combined_df = target_combined_df.drop("Unnamed: 0", axis=1)

    out_init_poses = []
    out_target_poses = []
    boxes_iter_count = 0
    start_t = time.time()
    for init_b, target_b in zip(init_boxes, target_boxes):
        pose_not_found_count = 0
        boxes_iter_count += 1
        while True:
            init_random_sample = get_rand_sample(init_combined_df,
                                                 bx=init_b[0],
                                                 by=init_b[1],
                                                 bz=init_b[2],
                                                 res_cm=min_dist_each_axis)
            target_random_sample = get_rand_sample(target_combined_df,
                                                   bx=target_b[0],
                                                   by=target_b[1],
                                                   bz=target_b[2],
                                                   res_cm=min_dist_each_axis)
            
            init_x = init_random_sample["Tx (m)"]
            init_y = init_random_sample["Ty (m)"]
            init_z = init_random_sample["Tz (m)"]

            target_x = target_random_sample["Tx (m)"]
            target_y = target_random_sample["Ty (m)"]
            target_z = target_random_sample["Tz (m)"]

            if (np.abs(init_x-target_x)*100 < min_dist_each_axis) or (np.abs(init_y-target_y)*100 < min_dist_each_axis) or (np.abs(init_z-target_z)*100 < min_dist_each_axis):
                pose_not_found_count += 1
            else:
                out_init_poses.append(list(init_random_sample))
                out_target_poses.append(list(target_random_sample))
                break

            if pose_not_found_count == max_iter_pose_before_exit:
                break

        if boxes_iter_count % 1000 == 0:
            print(f"num of boxes finished= {boxes_iter_count}; num of poses obtained = {len(out_init_poses)}; time elapsed = {time.time() - start_t:.2f} s")
    
    return out_init_poses, out_target_poses, cols


if __name__ == "__main__":
    # to vary dofs
    vary_dofs = ["5_dofs", "6_dofs", "6_plus_1_dof_trans", "6_plus_2_dofs_trans_vert", "6_plus_3_dofs", "table_3dofs"]
    vary_dofs = ["6_plus_2_dofs_trans_vert", "6_plus_3_dofs"]
    
    # poses_types
    pose_types = ["PA", "AP", "V1", "V2", "Ver", "Lat", "V1", "V2"]
    target_pose_types = ["PA", "AP", "V1", "V2", "Ver", "Lat", "V2", "V1"]

    # paths
    data_dir = "../workspace_analysis_outputs/out"
    out_root_dir = "../out_poses_gen"

    # multiple runs with varying dofs and pose types
    t1 = time.time()
    for v_dofs in vary_dofs:
        for p_type, target_p_type in zip(pose_types, target_pose_types):
            print(f"\n\n..... {v_dofs} ..... {p_type}.... {target_p_type}....\n")
            # initial poses
            initial_combined_csv_in_path = f"{data_dir}/{p_type}/{v_dofs}/c_arm_table_combined.csv"
            initial_sampled_csv_in_path = f"{data_dir}/{p_type}/{v_dofs}/c_arm_table_combined_sampled_gap_5cm.csv"

            # target poses
            target_combined_csv_in_path = f"{data_dir}/{target_p_type}/{v_dofs}/c_arm_table_combined.csv"
            target_sampled_csv_in_path = f"{data_dir}/{target_p_type}/{v_dofs}/c_arm_table_combined_sampled_gap_5cm.csv"

            out_dir = f"{out_root_dir}/{v_dofs}/{p_type}_{target_p_type}"
            out_initial_poses_csv_path = os.path.join(out_dir, "initial_poses.csv")
            out_target_poses_csv_path = os.path.join(out_dir, "target_poses.csv")
            if not os.path.isdir(out_dir):
                os.makedirs(out_dir)
            else:
                print("Out directory already exists. Please choose a different directory")
                sys.exit()

            # get initial and target boxes
            out_init_boxes, out_target_boxes = get_random_init_and_target_boxes(init_sampled_csv_path=initial_sampled_csv_in_path,
                                                                                target_sampled_csv_path=target_sampled_csv_in_path,
                                                                                num_of_out_poses=20000,
                                                                                max_iter_before_exit=100,
                                                                                min_dist_each_axis=5.0)

            # get initial and target poses
            init_poses, target_poses, col_names = get_rand_init_target_poses(initial_combined_csv_in_path, target_combined_csv_in_path, 
                                                                             init_boxes=out_init_boxes,
                                                                             target_boxes=out_target_boxes,
                                                                             max_iter_pose_before_exit=10,
                                                                             min_dist_each_axis=5.0)


            # save first 'n' poses
            num_of_poses_out = 15000

            out_initial_poses_df = pd.DataFrame(data=np.array(init_poses[0: num_of_poses_out]), columns=col_names)
            out_target_poses_df = pd.DataFrame(data=np.array(target_poses[0: num_of_poses_out]), columns=col_names)

            out_initial_poses_df.to_csv(out_initial_poses_csv_path)
            out_target_poses_df.to_csv(out_target_poses_csv_path)

            t2 = time.time()
            print(f"processed_time={round((t2-t1)/60, 2)} min")
            