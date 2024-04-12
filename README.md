# C_Arm_Workspace


## C_Arm + Table Workspace Analysis
- Code to perform collision free workspace analysis
- Workspace analysis: performed on six surgical poses; [reference](https://www.researchgate.net/profile/Nicolas-Loy-Rodas/publication/315768326_Pose_Optimization_of_a_C-Arm_Imaging_Device_to_Reduce_Intraoperative_Radiation_Exposure_of_Staff_and_Patient_During_Interventional_Procedures/links/59dcb1d3a6fdcc1ec8a75c16/Pose-Optimization-of-a-C-Arm-Imaging-Device-to-Reduce-Intraoperative-Radiation-Exposure-of-Staff-and-Patient-During-Interventional-Procedures.pdf)
    - Posterior-Anterior (PA)
    - Anterior-Posterior (AP)
    - Vascular 1 (V1)
    - Vascular 2 (V2)
    - Vertebroplasty (Ver)
    - Lateral (Lat)

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


