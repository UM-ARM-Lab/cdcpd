std_msgs/Header header
string[] gripper_names # N stings
geometry_msgs/Pose[] starting_object_configuration
geometry_msgs/Pose[] starting_gripper_poses
deformable_manipulation_msgs/VectorPose[] path_to_start_of_test # Path of length M, each with N poses
int32[] path_num_substeps # M entries
geometry_msgs/Pose[] final_gripper_targets # N entries
int32 final_num_substeps