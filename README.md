# C-arm_Table_Workspace

## Overview
In this project, C-arm and patient table combined setup with higher degrees of freedom (DOF) was studied to analyze the collision-free workspace for typical interventional configurations. A six DOF C-arm was designed using standard five C-arm joints and its movement on the floor parallel to the table as the 6th DOF. This C-arm is combined with a 5 DOF surgical table with assuming their initial pose. In this 11 DOF setup, two table rotational joints were ignored, and workspace analysis was performed on the remaining 9 DOF.

Forward kinematics for this setup was solved using DH parameters. Random joint configurations were selected within the joint limits to calculate the target pose and perform workspace analysis. Target pose is the pose of C-arm end-effector w.r.t table end-effector i.e., pose of C-arm w.r.t the patient on the table. Collisions between C-arm and table will occur for many joint configurations, this is detected by developing 3D meshes and point cloud representing our setup and identifying any intersections between these C-arm and table 3D objects. Lastly, a numerical iterative inverse kinematics was performed to analyze the collision-free workspace reachability with numerical solution compared to the actual reachability.

**For collision-free workspace results and further details about our setup, please refer to our paper (to be published).**

## C-arm Joints
Based on GE Healtcare's [OEC 3D isocentric C-arm](https://www.gehealthcare.com/products/surgical-imaging/oec-3d)
- Lateral (movement parallel to the patient table)
- Vertical
- WigWag
- Horizontal
- Tilt
- Orbital

## Table Joints
Based on Steris's [surgical patient table](https://www.steris.com/healthcare/products/surgical-tables/cmax-image-guided-surgical-table)
- Vertical
- Trend (Ignored) 
- Tilt (Ignored)
- Longitudinal
- Transverse

## Interventional Configurations
Workspace analysis was performed for [six typical surgical poses](https://www.researchgate.net/profile/Nicolas-Loy-Rodas/publication/315768326_Pose_Optimization_of_a_C-Arm_Imaging_Device_to_Reduce_Intraoperative_Radiation_Exposure_of_Staff_and_Patient_During_Interventional_Procedures/links/59dcb1d3a6fdcc1ec8a75c16/Pose-Optimization-of-a-C-Arm-Imaging-Device-to-Reduce-Intraoperative-Radiation-Exposure-of-Staff-and-Patient-During-Interventional-Procedures.pdf)
- Posterior-Anterior (PA)
- Anterior-Posterior (AP)
- Vascular 1 (V1)
- Vascular 2 (V2)
- Vertebroplasty (Ver)
- Lateral (Lat)

## Initial pose
C-arm and table initial pose is assumed such that a vertical line passes through the C-arm and table end-effectors when all joints are configured to zero except C-arm's horizontal and table's longitudinal joints configured to their centre values. This is to ensure from the initial pose there is movement possible on either directions.

![Figure_5](https://github.com/user-attachments/assets/0ac93305-9ee0-40f7-a6dd-0327f4449dc7)

## Collision Detection Examples
![Figure_9a](https://github.com/user-attachments/assets/6936b409-3157-43de-ba73-8f4474f81647) | ![Figure_9b](https://github.com/user-attachments/assets/a224795d-e771-4aab-8848-2bb304e3b0fb) | ![Figure_9c](https://github.com/user-attachments/assets/f7de7729-b08f-4770-b4d2-6167feef522f)
--- | --- | ---

## Code
- Install pip packages provided in requirements.txt file
  
### Forward Kinematics
- [Scripts/random_c_arm_table_joints_generate_11dofs.ipynb](Scripts/random_c_arm_table_joints_generate_11dofs.ipynb): to generate random joint configurations for 11 DOF
  - Inputs:
     - `num_of_samples`: assign with desired random samples
  - Outputs: **random_joint_values.csv** file with random joint configurations
 
- [forward_kinematics_gen_poses_main.py](forward_kinematics_gen_poses_main.py): calculate target poses for the input joint configurations
    - Inputs:
      - `input_joints_path`: path to csv file with joint configurations (output from the above step)
      - `out_poses_path`: output csv file path
    - Outputs: csv file with joint configurations and their target poses

- [ForwardKinematics-MatLab/c_arm_isocentric_plus_table_simscape_poses.m](ForwardKinematics-MatLab/c_arm_isocentric_plus_table_simscape_poses.m): calculate target poses for the input joint configurations using MatLab simscape
  - Inputs:
    - `joint_values_path`: path to csv file with joint configurations (same as the above step)
    - `out_poses_path`: output csv file path
  - Outputs: csv file with joint configurations and their target poses
   
- [Scripts/compare_c_arm_plus_table_poses_11dofs.m](Scripts/compare_c_arm_plus_table_poses_11dofs.m): compare python DH parameters and Matlab simscape output target poses
  - Inputs:
    - `csv_path_1`: path to output target poses csv file obtained from DH approach
    - `csv_path_2`: path to output target poses csv file obtained from Matlab simscape approach
    - `out_file`: output txt file path
  - Outputs: text file with translation and rotation errors
      
### Pose Visualization
- [ForwardKinematics-MatLab/main_live.mlx](ForwardKinematics-MatLab/main_live.mlx): MatLab live script to visualize a pose with option to change joint values
- [visualize_pose.py](visualize_pose.py): Python code to visualize a pose. Change joint values to your desired values.

### Workspace
- [workspace_analysis_main.py](workspace_analysis_main.py): generate random poses across the workspace and sample 5 cm x 5 cm x 5 cm boxes for performing workspace analysis
  - Inputs - to select different DOF setups: `vary_dofs`, `c_arm_joints_enable`, and `table_joints_enable`
  - Inputs - to select different interventional configurations: `pose_types`, `c_arm_tilt_pose_types`, and `c_arm_orbital_pose_types`
  - Outputs: random poses and sampled workspace (csv, html, metadata text files) for varying DOF setups for different interventional configurations  

### Target poses generation
- [workspace_gen_poses_main.py](workspace_gen_poses_main.py): generate random collision-free poses across the 7 DOF setup for different interventional configurations
  - Inputs:
    - `data_dir`: root directory of outputs obtained in the above step
    - `max_rand_poses_per_box`: maximum number of random collision-free poses to be selected per box. Configured to 10 in this study.
  - Outputs:csv file with target poses, one per interventional configuration.

- Target poses for 7 DOF setup
 
  Projection | No. of target poses
  --- | ---
  AP | 17,292
  PA | 17,278
  V1 | 16,583
  V2 | 42,610
  Ver | 18,733
  Lat | 26,717

### Inverse Kinematics
- [inverse_kinematics_main.py](inverse_kinematics_main.py): Inverse kinematics of target poses
  - Inputs:
      - `num_of_mp_threads`: number of parallel threads
      - `out_dir`: output directory path
      - `input_csv_path`: csv file with target poses (output from the above step)
      - `opt_jnts`: list of joints to solve. Other joints are configured to their initial pose. E.g. `['c_arm_lat', 'c_arm_vert', 'c_arm_wigwag', 'c_arm_horz', 'table_vert', 'table_long', 'table_trans']`
  - Outputs: csv file with inverse kinematics results

## Acknowledgement
This study was enabled in part by computational resources provided by Calcul Québec [(calculquebec.ca)](calculquebec.ca) and the Digital Research Alliance of Canada [(alliancecan.ca)](alliancecan.ca).
