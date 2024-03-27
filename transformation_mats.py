import numpy as np


# transformation matrix from table ee to c-arm ee
def get_transf_mat_table_ee_to_c_arm_ee(c_arm_lateral, c_arm_vertical, c_arm_wigwag, c_arm_horizontal, c_arm_tilt,
                                        c_arm_orbital, table_vertical, table_trend, table_tilt,
                                        table_longitudinal, table_transverse):
    # initial pose of c-arm base with table base
    transf_c_arm_base_to_table_base = np.eye(4)
    transf_c_arm_base_to_table_base[0, 3] = 0.4
    transf_c_arm_base_to_table_base[1, 3] = 1.575

    transf_c_arm_base_to_ee = calc_transf_mat_c_arm_base_to_ee(c_arm_lateral, c_arm_vertical, c_arm_wigwag,
                                                               c_arm_horizontal, c_arm_tilt, c_arm_orbital)

    transf_table_base_to_ee = calc_transf_mat_table_base_to_ee(table_vertical, table_trend, table_tilt,
                                                               table_longitudinal, table_transverse)

    transf_c_arm_base_to_table_ee = transf_c_arm_base_to_table_base @ transf_table_base_to_ee

    transf_table_ee_to_c_arm_base = get_inv_transformation_mat(transf_c_arm_base_to_table_ee)

    transf_table_ee_to_c_arm_ee = transf_table_ee_to_c_arm_base @ transf_c_arm_base_to_ee

    return transf_table_ee_to_c_arm_ee


# inverse of a transformation matrix
def get_inv_transformation_mat(transf_mat):
    r = transf_mat[0:3, 0:3]
    t = transf_mat[0:3, 3]
    inv_r = r.T
    inv_t = -inv_r @ t

    inv_transf = np.eye(4)
    inv_transf[0:3, 0:3] = inv_r
    inv_transf[0:3, 3] = inv_t

    return inv_transf


# table base to end-effector transformation matrix
def calc_transf_mat_table_base_to_ee(vertical, trend, tilt, longitudinal, transverse):
    # Joint1: base to vertical and coordinate system alignment for next joint
    l1 = 0.58  # base + link(elliptical) to the trend joint --> 0.36 + 0.22
    j1 = vertical  # 0 to 0.36
    theta1 = 0
    alpha1 = -np.pi / 2
    d1 = l1 + j1
    a1 = 0
    transf1 = get_transf_mat_dh_parameters(theta1, alpha1, d1, a1)

    # Joint2: trend joint and coordinate system alignment for the next joint
    j2 = trend  # -30 to 30
    theta2 = (-np.pi / 2) + (j2 * (np.pi / 180))
    alpha2 = np.pi / 2
    d2 = 0
    a2 = 0
    transf2 = get_transf_mat_dh_parameters(theta2, alpha2, d2, a2)

    # Joint3: tilt joint and coordinate system alignment for next joint
    j3 = tilt  # -20 to 20
    theta3 = j3 * (np.pi / 180)
    alpha3 = 0
    d3 = 0.05  # to align the floating top solid
    a3 = 0.025  # brick connecting elliptical with the floating top
    transf3 = get_transf_mat_dh_parameters(theta3, alpha3, d3, a3)

    # Joint4: longitudinal
    j4 = longitudinal  # 0 to 0.7
    theta4 = 0
    alpha4 = np.pi / 2
    d4 = j4
    a4 = 0  
    transf4 = get_transf_mat_dh_parameters(theta4, alpha4, d4, a4)

    # Joint5: transverse and shift axes to top of the table
    j5 = transverse  # -0.13 to 0.13
    theta5 = 0
    alpha5 = -np.pi / 2
    d5 = j5
    a5 = 0.075 # floating top thickness
    transf5 = get_transf_mat_dh_parameters(theta5, alpha5, d5, a5)

    table_transf_mat = transf1 @ transf2 @ transf3 @ transf4 @ transf5  # matrix multiplication

    return table_transf_mat


# calc transformation matrix using DH parameters
def get_transf_mat_dh_parameters(theta, alpha, d, a):
    c_t = np.cos(theta)
    c_a = np.cos(alpha)
    s_t = np.sin(theta)
    s_a = np.sin(alpha)

    # transf mat
    transf_mat = np.eye(4)
    transf_mat[0, 0] = c_t
    transf_mat[0, 1] = -s_t * c_a
    transf_mat[0, 2] = s_t * s_a
    transf_mat[0, 3] = a * c_t

    transf_mat[1, 0] = s_t
    transf_mat[1, 1] = c_t * c_a
    transf_mat[1, 2] = -c_t * s_a
    transf_mat[1, 3] = a * s_t

    transf_mat[2, 0] = 0
    transf_mat[2, 1] = s_a
    transf_mat[2, 2] = c_a
    transf_mat[2, 3] = d

    return transf_mat


# c-arm base to end-effector transformation matrix
def calc_transf_mat_c_arm_base_to_ee(lateral, vertical, wigwag, horizontal, tilt, orbital):
    # for new lateral joint - frame 1
    theta_n1 = np.pi/2
    alpha_n1 = np.pi/2
    d_n1 = 0
    a_n1 = 0
    transf_n1 = get_transf_mat_dh_parameters(theta_n1, alpha_n1, d_n1, a_n1)

    # for new lateral joint - frame 2
    theta_n2 = 0
    alpha_n2 = -np.pi/2
    d_n2 = lateral  # -0.25 to 0.55
    a_n2 = 0
    transf_n2 = get_transf_mat_dh_parameters(theta_n2, alpha_n2, d_n2, a_n2)

    # for new lateral joint - frame 3
    theta_n3 = -np.pi/2
    alpha_n3 = 0
    d_n3 = 0
    a_n3 = 0
    transf_n3 = get_transf_mat_dh_parameters(theta_n3, alpha_n3, d_n3, a_n3)

    # Joint1: base to vertical
    l1 = 0.99
    j1 = vertical  # 0 to 0.46
    theta1 = 0
    alpha1 = 0
    d1 = l1 + j1
    a1 = 0
    transf1 = get_transf_mat_dh_parameters(theta1, alpha1, d1, a1)

    # Joint2: wigwag and coordinate system align for next joint
    j2 = wigwag
    theta2 = j2 * (np.pi / 180)
    alpha2 = -np.pi / 2
    d2 = 0
    a2 = 0
    transf2 = get_transf_mat_dh_parameters(theta2, alpha2, d2, a2)

    # Joint3: horizontal
    j3 = horizontal  # 0 to 0.15
    l3 = 0.5
    theta3 = 0
    alpha3 = 0
    d3 = l3 + j3
    a3 = 0
    transf3 = get_transf_mat_dh_parameters(theta3, alpha3, d3, a3)

    # Joint4: tilt
    j4 = tilt  # -90 to 270
    theta4 = (-np.pi / 2) + (j4 * (np.pi / 180))
    alpha4 = np.pi / 2
    d4 = 0.75 + 0.1 + 0.15  # radius + thickness of orbital joint + tilt joint thickness
    a4 = 0
    transf4 = get_transf_mat_dh_parameters(theta4, alpha4, d4, a4)

    # Joint5: orbital
    j5 = orbital  # -100 to 100
    theta5 = j5 * (np.pi / 180)
    alpha5 = 0
    d5 = 0
    a5 = 0
    transf5 = get_transf_mat_dh_parameters(theta5, alpha5, d5, a5)

    # matrix multiplication
    c_arm_transf_mat = transf_n1 @ transf_n2 @ transf_n3 @ transf1 @ transf2 @ transf3 @ transf4 @ transf5

    return c_arm_transf_mat
