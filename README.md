# Visual Odometry (VO) with Deep Feature Matching

This project contains a simple C++ implementation of stereo 3D-3D and 3D-2D visual odometry algorithms for the KITTI dataset. Features for VO are extracted and matched by deep learning: the SuperPoint and SuperGlue networks from Magic Leap have been used. 

### Basic Usage

Building the C++ code requires to follow the usual CMake pipeline. The only dependencies are Eigen for linear algebra, Ceres for 3D-2D nonlinear optimization and RapidJson for parsing the configuration file. Running the code is as simple as:

```
./vo_3d3d <config_file>
./vo_3d2d <config_file>
```

The required input format is a set of four files for each frame in the sequence:

- `kp_0_%06d.txt` contains the list of keypoints for the left image as a pair of coordinates `(u,v)` for each line.
- `kp_1_%06d.txt` contains the list of keypoints for the right image as a pair of coordinates `(u,v)` for each line.
- `flow_%06d.txt` contains the monocular matches between frame `i` and frame `i+1` for the left image. The special value `65535` means that the keypoint is unmatched. 
- `stereo_%06d.txt` contains the stereo matches between left and right image of the same frame. The special value `65535` means that the keypoint is unmatched. 

### Feature Extraction and Matching

You can generate the required files with any method you want and run the code. However, I provide some Python scripts that generate them with deep learning. After having downloaded the dataset from the KITTI website (http://www.cvlibs.net/datasets/kitti/), the workflow is the following:

- Since the official codebase from Magic Leap allows to match a list of pairs of images, generate all the relevant (left -> right, prev -> cur) matches with:

```
python3 scripts/generate_matching_pairs.py --path <path_to_data> --sequence <num>
```

- Compute monocular matches between all the (prev -> cur) pairs for the left image with:

```
python3 scripts/match_pairs.py --resize -1 --superglue outdoor --max_keypoints 2048 --nms_radius 3  --resize_float --input_dir <path> --input_pairs <path>/prev_cur_pairs.txt --output_dir <path>/dump_prev_cur_matches --viz
```

- Computing stereo matches between left and right images for all pairs with:

```
python3 scripts/match_pairs.py --resize -1 --superglue outdoor --max_keypoints 2048 --nms_radius 3  --resize_float --input_dir <path> --input_pairs <path>/left_right_pairs.txt --output_dir <path>/dump_left_right_matches --viz
```

- Generate all the required files with:

```
python3 scripts/generate_odometry_files.py --path <path_to_data> --sequence <num>
```

You can inspect `.npz` matching files by simply loading them with NumPy.

### Visualizing Results

The C++ code will print the rotation and translation error after the execution. For visualizing the difference between the ground truth and the predicted trajectory, I provide a simple Python script:

```
python3 scripts/plot.py --pred <predicted_path> --gt <ground_truth_path>
```

### Acknowledgements

I give all the credits to the authors from Magic Leap (https://github.com/magicleap) for both the pretrained networks and the Python code for running inference. Also, some C++ evaluation files are heavily based on the official ones provided by the KITTI codebase.