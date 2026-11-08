#!/bin/bash
#SBATCH --job-name=label
#SBATCH --account=labi
#SBATCH --partition=a10
#SBATCH --gres=gpu:1
#SBATCH --mem=32G
#SBATCH --output=logs/job_%j.out
#SBATCH --error=logs/job_%j.err
#SBATCH --time=12:00:00

# SAM 3 road distillation pipeline (1 GPU max).
# Submit with:  sbatch run_sam3.sbatch [--stride 5 ...]
# Logs:         ./logs/job_<jobid>.out and ./logs/job_<jobid>.err
# Prerequisite: hf auth login (access to facebook/sam3) so the
#               gated checkpoint can download to the HF cache.

set -e

# NOTE: Slurm runs a *copy* of this script from /var/spool/slurm/...,
# so `dirname $0` points at the spool dir. Always return to the
# directory the job was submitted from.
cd "$SLURM_SUBMIT_DIR"
mkdir -p logs

export XDG_CACHE_HOME=/scratch/gilbreth/<YOUR USERNAME>

module load anaconda
conda activate Distill_v2

srun python main_sam3.py --stride 5 --clip-threshold 0.0 --sam-conf 0.5 "$@"
