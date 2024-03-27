#!/bin/bash
#SBATCH --time=05:00:00
#SBATCH --job-name=ws_df6
#SBATCH --output=ws_df6-%J.out
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --mem=1024

# Virtual environment
module load python/3.10
source ../../my_envs/workspace/bin/activate

python workspace_analysis_main6.py
