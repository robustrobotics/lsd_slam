# This message contains statistics from lsd_slam::SlamSystem.
std_msgs/Header header

float32 msTrackFrame
float32 msOptimizationIteration
float32 msFindConstraintsItaration
float32 msFindReferences
float32 nAvgTrackFrame
float32 nAvgOptimizationIteration
float32 nAvgFindConstraintsItaration
float32 nAvgFindReferences

# Load stats.
float32 max_load_cpu
float32 max_load_mem
float32 max_load_swap

float32 sys_load_cpu
float32 sys_load_mem
float32 sys_load_swap

float32 pid_load_cpu
float32 pid_load_mem
float32 pid_load_swap