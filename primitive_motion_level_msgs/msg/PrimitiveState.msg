string name
string parent_link_name
geometry_msgs/Pose local_pose

float64 time
geometry_msgs/Pose pose
float64[] wrench
float64[] pose_follow_gain
float64[] wrench_follow_gain

bool is_poseC_global
float64[] poseC
float64[] poseld
float64[] poseud
bool is_wrenchC_global
float64[] wrenchC
float64[] wrenchld
float64[] wrenchud

float64[] act_wrench
float64[] M
float64[] D
float64[] K

bool support_com
