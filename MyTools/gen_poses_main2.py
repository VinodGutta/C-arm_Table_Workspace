import sys
import numpy as np
import pandas as pd
import time
import os


# get target boxes
def get_target_boxes(target_sampled_csv_path):
    # read csv file as data frame
    target_sampled_df = pd.read_csv(target_sampled_csv_path, header=0, index_col=0)

    # collision free boxes
    target_collision_free_boxes = target_sampled_df[(target_sampled_df["Collision"] == 0.0) |
                                                    (target_sampled_df["Collision"] == 2.0)]
    target_collision_free_boxes_np = target_collision_free_boxes.to_numpy()
    
    return target_collision_free_boxes_np
    

# get a random poses from a box
def get_rand_poses_in_box(in_df, bx, by, bz, res_cm, max_poses_per_box):
    x1 = bx - (res_cm / 2)
    x2 = bx + (res_cm / 2)

    y1 = by - (res_cm / 2)
    y2 = by + (res_cm / 2)

    z1 = bz - (res_cm / 2)
    z2 = bz + (res_cm / 2)

    sample_poses = in_df[((in_df["Tx (m)"] >= (x1/100)) & (in_df["Tx (m)"] < (x2/100))) &
                         ((in_df["Ty (m)"] >= (y1/100)) & (in_df["Ty (m)"] < (y2/100))) &
                         ((in_df["Tz (m)"] >= (z1/100)) & (in_df["Tz (m)"] < (z2/100))) &
                         (in_df["end pose collision"] == 0)].to_numpy()

    total_poses_count = sample_poses.shape[0]
    # print(f"total_poses_count={total_poses_count}")

    if total_poses_count <= max_poses_per_box:
        return sample_poses
    
    rnd_idx = np.random.choice(np.arange(total_poses_count), 
                               size=max_poses_per_box,
                               replace=False)
    
    return sample_poses[rnd_idx, :]
 

# get random poses
def get_rand_target_poses(target_combined_csv_in_path, target_boxes, max_poses_per_box, box_dim_cm):
    print("Selecting random target poses...")

    # read csv file
    target_combined_df = pd.read_csv(target_combined_csv_in_path, header=0, index_col=0)
    target_combined_df = target_combined_df.drop("Unnamed: 0", axis=1)
    cols = target_combined_df.columns

    out_target_poses = []
    start_t = time.time()
    total_target_boxes = target_boxes.shape[0]
    for idx, target_b in enumerate(target_boxes):
        rand_box_poses = get_rand_poses_in_box(target_combined_df,
                                               bx=target_b[0],
                                               by=target_b[1],
                                               bz=target_b[2],
                                               res_cm=box_dim_cm,
                                               max_poses_per_box=max_poses_per_box)
        out_target_poses.append(rand_box_poses)

        if (idx + 1)%100 == 0:
            time_elapsed = time.time() - start_t
            print(f"processed: {idx + 1}/{total_target_boxes} boxes; time_elapsed={time_elapsed:.2f} s")
    
    return np.concatenate(out_target_poses, axis=0), cols

        
if __name__ == "__main__":
    # to vary dofs
    v_dofs = "6_plus_3_dofs"
    
    # poses_types
    pose_types = ["PA", "AP", "V1", "V2", "Ver", "Lat"]

    # box changes
    max_rand_poses_per_box = 10
    box_dim_cm = 5.0

    # paths
    data_dir = "./out_WS"
    out_root_dir = "./poses_gen_v2"

    # multiple runs with varying dofs and pose types
    t1 = time.time()
    total_poses_count = []
    for p_type in pose_types:
        print(f"\n\n{p_type}..... \n")
        # poses
        combined_csv_in_path = f"{data_dir}/{p_type}/{v_dofs}/c_arm_table_combined.csv"
        sampled_csv_in_path = f"{data_dir}/{p_type}/{v_dofs}/c_arm_table_combined_sampled_gap_5cm.csv"

        out_dir = f"{out_root_dir}/{p_type}"
        out_target_poses_csv_path = os.path.join(out_dir, "target_poses.csv")
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        else:
            print("Out directory already exists. Please choose a different directory")
            sys.exit()

        # get target boxes
        out_target_boxes = get_target_boxes(sampled_csv_in_path)

        # get random poses
        data, cols = get_rand_target_poses(combined_csv_in_path, 
                                           out_target_boxes, 
                                           max_rand_poses_per_box,
                                           box_dim_cm)
        
        # save csv
        out_target_poses_df = pd.DataFrame(data=data, columns=cols)
        out_target_poses_df.to_csv(out_target_poses_csv_path)
        t2 = time.time()
        poses_count = out_target_poses_df.shape[0]
        total_poses_count.append(poses_count)
        print(f"poses_count={out_target_poses_df.shape[0]}; processed_time={round((t2-t1)/60, 2)} min")
    
    print(f"total num of poses = {np.array(total_poses_count).sum()}")