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
- 

## Folders and files 
- 3d_inputs: 3D watertight meshes and pointcloud files (for collision detection)
- MyTools: python scripts
    - gen_poses_main.py: code to generate random initial and target collision free poses
        - Random poses were generated for two configuration. 1) C-arm (6 dofs) + Table (2 dofs); 2) C-arm (6 dofs) + Table (3 dofs). For each configuration,
            - Generated initial and target poses = 120,000
            - Initial pose = PA; Target pose = PA; generated samples = 15,000
            - Initial pose = AP; Target pose = AP; generated samples = 15,000
            - Initial pose = V1; Target pose = V1; generated samples = 15,000
            - Initial pose = V2; Target pose = V2; generated samples = 15,000
            - Initial pose = Ver; Target pose = Ver; generated samples = 15,000
            - Initial pose = Lat; Target pose = Lat; generated samples = 15,000
            - Initial pose = V1; Target pose = V2; generated samples = 15,000
            - Initial pose = V2; Target pose = V1; generated samples = 15,000
    
    - read_workspace_boxes.py: script to read workspace boxes analysis metadata.

- workspace_analysis_main.py: Main code to generate workspace analysis files with varying dofs.
    - Imports necessary code from other files such as transformation_mats.py, position_plot.py, and workspace_sampling.py files.
- visualize_pose.py: Code to visualize collision detection for a custom joint values.


