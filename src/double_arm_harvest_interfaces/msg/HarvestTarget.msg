# One harvest target for a single arm.
#
# The execution layer receives the full pose directly; a future vision node is
# responsible for converting detections into PoseStamped in the base frame.
#
# Constraints enforced by the executor:
#   - target_pose.header.frame_id must equal the configured base frame (v1: base_link)
#   - the quaternion must be finite and normalised (small errors are re-normalised)
#   - confidence must lie in [0, 1]
#   - target_id identifies the task for duplicate rejection and result logging
string target_id
geometry_msgs/PoseStamped target_pose
float32 confidence
string source
