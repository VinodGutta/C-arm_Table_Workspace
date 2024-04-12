import sys
import numpy as np
import pandas as pd
import time
import os


def get_rand_sample(in_df, bx, by, bz, res_cm=5.0):
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


def get_init_target_poses(initial_combined_csv_in_path, initial_sampled_csv_in_path,
                          target_combined_csv_in_path, target_sampled_csv_in_path,
                          num_of_poses):
    # read csv files
    init_combined_df = pd.read_csv(initial_combined_csv_in_path, header=0, index_col=0)
    init_combined_df = init_combined_df.drop("Unnamed: 0", axis=1)
    init_sampled_df = pd.read_csv(initial_sampled_csv_in_path, header=0, index_col=0)

    target_combined_df = pd.read_csv(target_combined_csv_in_path, header=0, index_col=0)
    target_combined_df = target_combined_df.drop("Unnamed: 0", axis=1)
    target_sampled_df = pd.read_csv(target_sampled_csv_in_path, header=0, index_col=0)

    # collision free boxes
    init_collision_free_boxes = init_sampled_df[(init_sampled_df["Collision"] == 0.0) |
                                                (init_sampled_df["Collision"] == 2.0)]

    target_collision_free_boxes = target_sampled_df[(target_sampled_df["Collision"] == 0.0) |
                                                    (target_sampled_df["Collision"] == 2.0)]

    initial_poses = []
    target_poses = []

    if (init_collision_free_boxes.shape[0] == 0) or (target_collision_free_boxes.shape[0] == 0):
        initial_poses = [[None]*len(init_combined_df.columns)]
        target_poses = [[None] * len(init_combined_df.columns)]
        return initial_poses, target_poses, init_combined_df.columns

    while True:
        for init_rand_box_idx in range(0, init_collision_free_boxes.shape[0]):
            x1 = init_collision_free_boxes.iloc[init_rand_box_idx][0]
            y1 = init_collision_free_boxes.iloc[init_rand_box_idx][1]
            z1 = init_collision_free_boxes.iloc[init_rand_box_idx][2]

            while True:
                target_rand_box_idx = np.random.randint(0, target_collision_free_boxes.shape[0])

                x2 = target_collision_free_boxes.iloc[target_rand_box_idx][0]
                y2 = target_collision_free_boxes.iloc[target_rand_box_idx][1]
                z2 = target_collision_free_boxes.iloc[target_rand_box_idx][2]

                if abs(x1-x2) >= 5.0 or abs(y1-y2) >= 5.0 or abs(z1-z2) >= 5.0:
                    break

            init_rand_sample = get_rand_sample(init_combined_df, bx=x1, by=y1, bz=z1)
            target_rand_sample = get_rand_sample(target_combined_df, bx=x2, by=y2, bz=z2)

            # check if the random sample already exists in the list
            if list(init_rand_sample) in initial_poses:
                init_pose_idx = initial_poses.index(list(init_rand_sample))
                target_pose_temp = target_poses[init_pose_idx]
                if target_pose_temp != list(target_rand_sample):
                    initial_poses.append(list(init_rand_sample))
                    target_poses.append(list(target_rand_sample))
            else:
                initial_poses.append(list(init_rand_sample))
                target_poses.append(list(target_rand_sample))

            if len(initial_poses) % 1000 == 0:
                print(f"generated poses = {len(initial_poses)}")

            if len(initial_poses) == num_of_poses:
                break

        if len(initial_poses) == num_of_poses:
            break

    return initial_poses, target_poses, init_combined_df.columns


if __name__ == "__main__":
    # to vary dofs
    vary_dofs = ["5_dofs", "6_dofs", "6_plus_1_dof_trans", "6_plus_2_dofs_trans_vert", "6_plus_3_dofs", "table_3dofs"]
    
    # poses_types
    pose_types = ["PA", "AP", "V1", "V2", "Ver", "Lat", "V1", "V2"]
    target_pose_types = ["PA", "AP", "V1", "V2", "Ver", "Lat", "V2", "V1"]

    # paths
    data_dir = "./out_workspace_analysis"
    out_dir = "./out_poses_gen"

    # multiple runs with varying dofs and pose types
    t1 = time.time()
    for v_dofs in vary_dofs:
        for p_type, target_p_type in zip(pose_types, target_pose_types):
            print(f"..... {v_dofs} ..... {p_type}.... {target_p_type}....")
            # initial poses
            initial_combined_csv_in_path = f"{data_dir}/{p_type}/{v_dofs}/c_arm_table_combined.csv"
            initial_sampled_csv_in_path = f"{data_dir}/{p_type}/{v_dofs}/c_arm_table_combined_sampled_gap_5cm.csv"

            # target poses
            target_combined_csv_in_path = f"{data_dir}/{target_p_type}/{v_dofs}/c_arm_table_combined.csv"
            target_sampled_csv_in_path = f"{data_dir}/{target_p_type}/{v_dofs}/c_arm_table_combined_sampled_gap_5cm.csv"

            out_dir = f"{out_dir}/{v_dofs}/{p_type}_{target_p_type}"
            out_initial_poses_csv_path = os.path.join(out_dir, "initial_poses.csv")
            out_target_poses_csv_path = os.path.join(out_dir, "target_poses.csv")
            if not os.path.isdir(out_dir):
                os.makedirs(out_dir)
            else:
                print("Out directory already exists. Please choose a different directory")
                sys.exit()

            init_poses, target_poses, cols = get_init_target_poses(initial_combined_csv_in_path,
                                                                    initial_sampled_csv_in_path,
                                                                    target_combined_csv_in_path,
                                                                    target_sampled_csv_in_path,
                                                                    num_of_poses=15000)

            out_initial_poses_df = pd.DataFrame(data=np.array(init_poses), columns=cols)
            out_target_poses_df = pd.DataFrame(data=np.array(target_poses), columns=cols)

            out_initial_poses_df.to_csv(out_initial_poses_csv_path)
            out_target_poses_df.to_csv(out_target_poses_csv_path)

            t2 = time.time()
            print(f"processed_time={round((t2-t1)/60, 2)} min")
