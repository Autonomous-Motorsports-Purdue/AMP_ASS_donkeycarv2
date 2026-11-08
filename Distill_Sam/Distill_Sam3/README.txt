Code for using SAM 3 to automatically label video frames for model fine tuning.

Usage:
- Build the conda environment from the environment.yml file
- Go to huggingface and ask for permission to use SAM 3(it is a gated model)
- Set up io folders (or symlinks)
    - Have a folder called videos which holds videos from the kart's camera
    - Have a folder called labels which will hold the segmented mask from SAM 3
    - Have a folder called labelled which will hold the raw video frame that SAM 3 labelled with a matching file name
- Run the code, this can be done with run_sam3.sbatch
    - videos argument to specify input directory
    - labelled and labels argument for output directories
    - save-overlay will determine if overlaid images are saved (for debugging)
    - overlay folder will hold images with a blue mask overlaid, can be used for debugging
    - stride will allow some frames to be skipped to reduce similar frames
    - state is where a txt file that tracks the videos that are finished being processed, used to resume this code when a job dies on gilbreth
    - onnx argument points to an old segmentation model, we take a point near the middle of the mask and use it as a starting point to help SAM 3
    - The other arguments arent that important and it's vibe coded anyway
- After the labels and labelled images are obtained go to TwinLiteNetPlus and finetune the model