# unstructured-robotic-grasping-ros
Robotic Pick and Place in Python in Unstructured Environments

# INITIAL VERSION (just vision):
- roslaunch zed_wrapper zed_no_tf.launch
- roslaunch yolov5_ros yolov5_camera.launch 
- user@zedbox:~/catkin_ws/src/yolov5_ros/src$ python obj_detect_ciccio.py

-----------------------------------------------------------------------------------------------

NOTE for GEOMETRICAL METHODS:
zed-ros-wrapper>zed_wrapper>"common.yaml": resolution VGA (ID value: 3)

----------------------------------------------------------------------------------------------

# GEOMETRICAL METHOD #1: TATA 
- in zedbox:
	- roslaunch zed_wrapper zed_no_tf.launch
	- roslaunch yolov5_ros yolov5_camera.launch ( source from camera, no yolov5.launch)
	- python pose_estimator_geometrical_methods.py
- in nuc:
	- 02_pp_TATA_method.py

-----------------------------------------------------------------------------------------------

# GEOMETRICAL METHOD #2: TEA - TRIGONOMETRY ESTIMATED ANGLE
- in zedbox:
	- roslaunch zed_wrapper zed_no_tf.launch
	- roslaunch yolov5_ros yolov5_camera.launch ( source from camera, no yolov5.launch)
	- python pose_estimator_geometrical_methods.py
- in nuc:
	- 03_0_pp_estimated_angle_bottle_proportion.py (works better but it is specific for bottle's aspect ratio)
	OR
	- 03_1_pp_estimated_angle_three_steps.py

------------------------------------------------------------------------------------------------

# GEOMETRICAL METHOD #3: SOFTWARE IMAGE ROTATION
- in zedbox (in order):
	- python sw_image_generator.py
	- roslaunch yolov5_ros yolov5.launch (subscribed to a fake topic, not real zed images)
	- python sw_rotation.py (kill and run it again for a second pick)
	- python pose_estimator_geometrical_methods.py
- in nuc:
	- python 04_pp_software_rotation.py

------------------------------------------------------------------------------------------------

# SPOT: Segmentation PCA based Object Tracking

PREDEFINED SETUP: resolution 2 (common.yaml)

FIRST VERSION with multiple picking:
- roslaunch zed_wrapper zed_no_tf.launch
- conda activate yolov8_env, poi "python yolo_seg_pca_ros_zedcam_RVIZ.py"
- python pose_estimator_tf.py
- (NUC): python 051_pick_place_official_version_v8_multiple_objects.py (multiple picking with loop in P&P script)

TRACKING VERSION: last version with tracking (some issues due to the tracker)
- yolo_seg_pca_ros_zedcam_RVIZ_multiple_predictions_tracking.py
- pose_estimator_tf_multiple_track.py
- 054_pick_place_official_version_v8_multiple_objects_tracking_workspace.py
