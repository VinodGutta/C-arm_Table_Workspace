import plotly.express as px
import pandas as pd
import numpy as np
import time


def sample_workspace(in_csv_path, res_cm=1.0):
    out_csv = in_csv_path[:-4] + f"_sampled_gap_{int(res_cm)}cm.csv"
    out_html = in_csv_path[:-4] + f"_sampled_gap_{int(res_cm)}cm.html"
    out_txt = in_csv_path[:-4] + f"_sampled_gap_{int(res_cm)}cm.txt"

    # read position n
    df = pd.read_csv(in_csv_path, header=0, index_col=0)
    pos_df = df[['Tx (m)', 'Ty (m)', 'Tz (m)', 'end pose collision']]

    # convert units from metres to cm
    pos_df.columns = ["Tx_cm", "Ty_cm", "Tz_cm", "Collision"]
    pos_df = pos_df.mul({"Tx_cm": 100, "Ty_cm": 100, "Tz_cm": 100, "Collision": 1})

    # minimum and maximum values in xyz axis
    x_min = np.floor(pos_df["Tx_cm"].to_numpy().min())
    x_max = np.floor(pos_df["Tx_cm"].to_numpy().max())

    y_min = np.floor(pos_df["Ty_cm"].to_numpy().min())
    y_max = np.floor(pos_df["Ty_cm"].to_numpy().max())

    z_min = np.floor(pos_df["Tz_cm"].to_numpy().min())
    z_max = np.floor(pos_df["Tz_cm"].to_numpy().max())

    # sampling 1 cm x 1 cm x 1cm boxes
    # new categories; 0 = no collision, 1 = collision, 2 = both, 3 = gap
    sampled_data = []
    total_boxes = ((x_max - x_min) / res_cm) * ((y_max - y_min) / res_cm) * ((z_max - z_min) / res_cm)
    count = 0
    t1 = time.time()
    sample_t1 = time.time()
    for box_x1 in np.arange(x_min, x_max, res_cm):
        for box_y1 in np.arange(y_min, y_max, res_cm):
            for box_z1 in np.arange(z_min, z_max, res_cm):
                count += 1
                if count % 50000 == 0:
                    sample_t2 = time.time()
                    print(f"processing = {count}/{total_boxes}; "
                          f"time_elapsed_min={round((sample_t2 - sample_t1) / 60, 3)}")
                box_x2 = box_x1 + res_cm
                box_y2 = box_y1 + res_cm
                box_z2 = box_z1 + res_cm

                filt_box = pos_df[((pos_df["Tx_cm"] >= box_x1) & (pos_df["Tx_cm"] < box_x2)) &
                                  ((pos_df["Ty_cm"] >= box_y1) & (pos_df["Ty_cm"] < box_y2)) &
                                  ((pos_df["Tz_cm"] >= box_z1) & (pos_df["Tz_cm"] < box_z2))]

                box_x = (box_x1 + box_x2) / 2
                box_y = (box_y1 + box_y2) / 2
                box_z = (box_z1 + box_z2) / 2

                if not filt_box.empty:
                    collision_np = filt_box["Collision"].to_numpy()
                    cat = None
                    if (0 in collision_np) and (1 in collision_np):
                        cat = 2
                    elif 0 in collision_np:
                        cat = 0
                    elif 1 in collision_np:
                        cat = 1
                else:
                    cat = 3

                sampled_data.append([box_x, box_y, box_z, cat])

    # sampled dataframe
    sampled_data_np = np.array(sampled_data)
    sampled_df = pd.DataFrame(data=sampled_data_np, columns=["box_center_x (cm)", "box_center_y (cm)",
                                                             "box_center_z (cm)", "Collision"])
    sampled_df.to_csv(out_csv)

    # save html
    sampled_df_html = sampled_df.copy()
    sampled_collision_np = sampled_df["Collision"].to_numpy()

    sampled_label_np = np.array([None] * sampled_collision_np.shape[0])
    sampled_label_np[sampled_collision_np == 1] = 'collision'
    sampled_label_np[sampled_collision_np == 0] = 'no_collision'
    sampled_label_np[sampled_collision_np == 2] = 'both'
    sampled_label_np[sampled_collision_np == 3] = 'gap'

    sampled_df_html["Collision"] = sampled_label_np

    fig = px.scatter_3d(sampled_df_html, x='box_center_x (cm)', y='box_center_y (cm)', z='box_center_z (cm)',
                        color='Collision', color_discrete_map={'collision': 'red',
                                                               'no_collision': 'blue',
                                                               'both': 'yellow',
                                                               'gap': 'green'})
    fig.write_html(out_html)

    # save txt
    no_collision_boxes_count = np.count_nonzero(sampled_collision_np == 0)
    collision_boxes_count = np.count_nonzero(sampled_collision_np == 1)
    both_boxes_count = np.count_nonzero(sampled_collision_np == 2)
    gap_boxes_count = np.count_nonzero(sampled_collision_np == 3)
    total_boxes = sampled_collision_np.shape[0]
    out_txt_f = open(out_txt, 'w')
    out_txt_f.write(f"no_collision_boxes_count={no_collision_boxes_count}\n")
    out_txt_f.write(f"collision_boxes_count={collision_boxes_count}\n")
    out_txt_f.write(f"both_boxes_count={both_boxes_count}\n")
    out_txt_f.write(f"gap_boxes_count={gap_boxes_count}\n")
    out_txt_f.write(f"total_boxes={total_boxes}\n")
    out_txt_f.close()

    t2 = time.time()
    print(f"Finished Sampling; time_elapsed_min={round((t2 - t1) / 60, 3)}")

