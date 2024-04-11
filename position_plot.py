import plotly.express as px
import pandas as pd
import numpy as np
import os


def save_position_plot(in_csv_path, out_dir):
    out_html = os.path.join(out_dir, in_csv_path.split("/")[-1].split(".")[0] + ".html")
    out_txt = os.path.join(out_dir, in_csv_path.split("/")[-1].split(".")[0] + ".txt")

    df = pd.read_csv(in_csv_path, header=0, index_col=0)
    pos_df = df[['Tx (m)', 'Ty (m)', 'Tz (m)', 'end pose collision']]

    pos_x_cm = pos_df['Tx (m)'].to_numpy() * 100
    pos_y_cm = pos_df['Ty (m)'].to_numpy() * 100
    pos_z_cm = pos_df['Tz (m)'].to_numpy() * 100
    collision_np = pos_df['end pose collision'].to_numpy()

    pos_x_cm_df = pd.DataFrame({'Tx (cm)': pos_x_cm})
    pos_y_cm_df = pd.DataFrame({'Ty (cm)': pos_y_cm})
    pos_z_cm_df = pd.DataFrame({'Tz (cm)': pos_z_cm})

    label_np = np.array([None] * collision_np.shape[0])
    label_np[collision_np == 1] = 'collision'
    label_np[collision_np == 0] = 'no_collision'
    label_df = pd.DataFrame({'label': label_np})

    final_df = pd.concat([pos_x_cm_df, pos_y_cm_df, pos_z_cm_df, label_df], axis=1, join='inner')

    # metadata
    x = final_df['Tx (cm)']
    y = final_df['Ty (cm)']
    z = final_df['Tz (cm)']

    x_no_colli = final_df['Tx (cm)'][collision_np == 0]
    y_no_colli = final_df['Ty (cm)'][collision_np == 0]
    z_no_colli = final_df['Tz (cm)'][collision_np == 0]

    # save txt
    out_f = open(out_txt, 'w')
    out_f.write(f"Total_Samples={x.shape[0]}\n")
    out_f.write(f"Collision_Samples={label_np[collision_np == 1].shape[0]}\n")
    out_f.write(f"No_Collision_Samples={label_np[collision_np == 0].shape[0]}\n")
    out_f.write(f"x_min={round(x.min(), 3)} cm; x_max={round(x.max(), 3)}  cm\n")
    out_f.write(f"y_min={round(y.min(), 3)}; y_max={round(y.max(), 3)}  cm\n")
    out_f.write(f"z_min={round(z.min(), 3)}; z_max={round(z.max(), 3)}  cm\n")
    out_f.write(
        f"No_collision --> x_min={round(x_no_colli.min(), 3)}  cm; x_max={round(x_no_colli.max(), 3)}  cm\n")
    out_f.write(
        f"No_collision --> y_min={round(y_no_colli.min(), 3)}  cm; y_max={round(y_no_colli.max(), 3)}  cm\n")
    out_f.write(
        f"No_collision --> z_min={round(z_no_colli.min(), 3)}  cm; z_max={round(z_no_colli.max(), 3)}  cm\n")
    out_f.close()

    # color_discrete_sequence = ['red', 'blue']
    fig = px.scatter_3d(final_df, x='Tx (cm)', y='Ty (cm)', z='Tz (cm)', color='label',
                        color_discrete_map={'collision': 'red', 'no_collision': 'blue'})
    # # fig.show()
    fig.write_html(out_html)
