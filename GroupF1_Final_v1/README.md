# Final Demonstration
#### Group F1: Ishaan Guha Chowdhary, Yee Kang Er, Jason Grant Angus, Imran Hagi
## Order to Run:
1. operate_slam.py : Create SLAM map and predict
2. TargetPoseEst.py : Estimate positions based on pred
3. slam_fruit.py : Combine Maps
4. pathFinder.py : Create path
5. auto_fruit_search.py : Autonomous Run

## Important Requirements:

### <b>File Names:</b>
- shopping list -> shopping_list.txt
- estimated SLAM map -> lab_output/slam.txt
- targets -> lab_output/targets.txt
- combined map -> slam_generated_map.txt
- outputted route -> output_route.txt

### <b>Dependencies:</b>

### operate_slam.py
- lines 154-162 : EKF Dependencies (intrinsic,distcoeffs,scale,baseline)
- lines 310-317 : Arguments (parameters, YOLO model)

### operate.py
- lines 154-162 : EKF Dependencies (intrinsic,distcoeffs,scale,baseline)
- lines 310-317 : Arguments (parameters, YOLO model weights)

### TargetPoseEst.py
- line 133 : intrinsic.txt
- line 137 : YOLO model weights
- line 142 : images.txt
- line 169 : targets.txt

### slam_fruit.py
- line 5 : slam.txt, targets.txt
- line 23 : slam_generated_map.txt

### pathFinder.py
- line 319 : slam_generated_map.txt
- line 320 : shopping_list.txt
- line 402 : output_route.txt

### auto_fruit_search.py
- line 109 : scale.txt
- line 111 : baseline.txt
- line 229 : slam_generated_map.txt
- lines 231-239 : Arguments (parameters, YOLO model weights)
- line 265 : shopping_list.txt
- line 283 : output_route.txt