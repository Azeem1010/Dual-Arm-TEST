# Calibration procedure

All six shipped sim/hardware records are NOT_RUN and contain null matrices. They cannot authorize motion. Camera detection tests do not change that status.

1. Record camera serial, resolution, optical frame, mounting revision and image-processing rotation/undistortion settings. For eye-in-hand cameras record Home joint feedback and camera-to-TCP transform. Fixed simulation cameras are deliberately labeled fixed, not hand-eye calibrated hardware.
2. Run `ros2 run vision_pkg calibrate_intrinsics --help`, acquire multiple views of a measured chessboard with image evidence. Use the original upstream calibration tool; retain observations, distortion model and reprojection errors.
3. Define world +X left-to-right C/B/A, +Y away from user, +Z upward. Slots 1/2 are nearest the user, left/right. Measure slot centers, board/target surface height, hole width, cap and cup diameter, mechanical repeatability and margin.
4. Run `ros2 run vision_pkg calibrate_homography --help`. Calibrate at the actual target plane and exact image preprocessing. Arm mappings output metres in their base frame; a_station mapping outputs millimetres. Do not mix units.
5. Use at least four additional noncollinear points not used for fitting. Run `PYTHONPATH=src/dual_arm_vision_adapter python3 src/dual_arm_vision_adapter/calibration/validate_plane.py measurements.json report.json`. The report compares maximum held-out error with `min((hole-cap)/2,(cap-cup)/2)-mechanical_error-margin`. Inputs must be measured; a nonpositive budget rejects the setup.
6. Independently validate TF2, image resolution, projected slot numbering, pickup surface height and mount/pose signature. For moving wrist cameras use measured hand-eye plus current joint TF, or restrict plane mapping to its calibrated Home pose. A name such as Home_A alone is not pose evidence.
7. Store timestamps in an explicitly documented clock domain, expiry, input evidence hashes and verification results. Keep simulation and hardware directories separate. The status monitor verifies evidence files but a human-edited PASS flag alone is not sufficient engineering acceptance.
8. Revoke validity on any mount, image rotation, resolution, focus or pose change. Require fresh image acquisition timestamps for every target; upstream service responses have no stamps and topic stamps are publication time.

Simulation acceptance requires actual rendered sensor images, CameraInfo, TF and independently compared Gazebo ground truth. Synthetic OpenCV fixtures remain unit tests. Physical calibration and hand-eye acquisition are NOT_RUN until devices are connected.
