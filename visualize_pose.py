import numpy as np
from transformation_mats import calc_transf_mat_c_arm_base_to_ee, calc_transf_mat_table_base_to_ee
import vedo


if __name__ == "__main__":
    # c-arm joint values
    c_arm_lateral = 0.523936  # c-arm lateral --> -0.25 to 0.55
    c_arm_vertical = 0.151159  # c-arm vertical --> 0 to 0.46
    c_arm_wigwag = -9.981029  # c-arm wigwag --> -10 to 10
    c_arm_horizontal = 0.124079  # c-arm horizontal --> 0 to 0.15
    c_arm_tilt = 30  # c-arm tilt --> -90 to 270
    c_arm_orbital = 30  # c-arm orbital --> -100 to 100

    # table joint values
    table_vertical = 0.160316  # table vertical --> 0 to 0.36
    table_trend = 0  # table trend --> -30 to 30
    table_tilt = 0  # table tilt --> -20 to 20
    table_longitudinal = 0  # table longitudinal --> 0 to 0.7
    table_transverse = -0.111679  # table vertical --> -0.13 to 0.13

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
        print("Collision detected")
        end_pose_collision = 1
    else:
        print("No collision detected")

    # visualize
    plt = vedo.Plotter()
    plt.show(table_top_mesh_cpy, table_body_mesh_cpy, table_wheels_base_mesh_cpy, c_arm_pc_cpy, axes=1).close()
    # sys.exit()



