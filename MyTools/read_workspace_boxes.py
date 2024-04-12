import pandas as pd
import numpy as np
import os


if __name__ == "__main__":
    # to vary dofs
    vary_dofs = ["5_dofs", "6_dofs", "6_plus_1_dof_trans", "6_plus_2_dofs_trans_vert", "6_plus_3_dofs", "table_3dofs"]

    # poses_types
    pose_types = ["PA", "V1", "V2", "Ver", "Lat"]

    out_data = {"pose":[], "dofs":[], "no_collision_boxes_count":[], "collision_boxes_count":[],
                "both_boxes_count":[], "gap_boxes_count":[], "total_boxes":[]}

    for p in pose_types:
        for v_dofs in vary_dofs:
            in_txt_path = f"./out/{p}/{v_dofs}/c_arm_table_combined_sampled_gap_5cm.txt"
            f = open(in_txt_path, 'r')
            f_lines = f.readlines()
            for line in f_lines:
                line_split = line.split("=")
                if line_split[0] == "no_collision_boxes_count":
                    no_collision_boxes_count = int(line_split[1])
                if line_split[0] == "collision_boxes_count":
                    collision_boxes_count = int(line_split[1])
                if line_split[0] == "both_boxes_count":
                    both_boxes_count = int(line_split[1])
                if line_split[0] == "gap_boxes_count":
                    gap_boxes_count = int(line_split[1])
                if line_split[0] == "total_boxes":
                    total_boxes = int(line_split[1])

            # data
            out_data['pose'].append(p)
            out_data['dofs'].append(v_dofs)
            out_data['no_collision_boxes_count'].append(no_collision_boxes_count)
            out_data['collision_boxes_count'].append(collision_boxes_count)
            out_data['both_boxes_count'].append(both_boxes_count)
            out_data['gap_boxes_count'].append(gap_boxes_count)
            out_data['total_boxes'].append(total_boxes)

    # df
    out_df = pd.DataFrame.from_dict(out_data)
    out_df.to_csv("poses_dofs_boxes_metadata.csv")
