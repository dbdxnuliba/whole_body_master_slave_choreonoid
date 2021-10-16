string name
string parent_link_name
geometry_msgs/Pose local_pose

float64 time
geometry_msgs/Pose pose
float64[] wrench

float64[] poseC
float64[] poseld
float64[] poseud
float64[] wrenchC
float64[] wrenchld
float64[] wrenchud
bool follow_pose
bool follow_wrench

float64[] act_wrench
float64[] wrench_gain
float64[] M
float64[] D
float64[] K

bool support_com
