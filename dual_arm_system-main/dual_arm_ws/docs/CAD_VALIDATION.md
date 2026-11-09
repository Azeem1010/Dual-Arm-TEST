# CAD validation

Source: pinned dual_arm_3d STEP, exported using OpenCascade 8.0.1.1.0. 264 leaf components are retained as binary STL, total approximately 46 MB. Source CAD files are unchanged.

PASS: original assembly pose preserved for all 264 visual components within 1e-9 transform tolerance (`scripts/test_cad_pose.py`, evidence/cad_pose_validation.json). Eight output axes are extracted from motor cylindrical faces, rather than inferred from mesh bounding boxes. Each arm has four independent revolute joints. The original PhantomX reference macro remains available separately.

NOT_RUN: measured mass/inertia, hardware joint travel, motor sign/zero, fastener rigid-body membership through motion, collision-free task paths and full CAD visual acceptance. Fasteners use nearest main-component membership and require review. Simulation inertials and ±1.5 rad limits are explicitly provisional. The fixed source pose is q=0 in the CAD model; it is not the CM530 HOME calibration.

CAD coordinate conversion: world = Rx(90°) * CAD_mm / 1000 + [0,0.03,0]. World X is left-to-right, Y away from user, Z up. Source platform span is 600 × 300 mm with front notches. A distinct 600 × 300 mm foam layer is modeled above it to meet the vision specification; this layer is simulation fixture geometry, not a change to source CAD.

Actual CAD hole centers in world metres: x = -0.1675 / -0.1325; y = -0.015, 0.020, 0.055, 0.090. Hole width is 25 mm. Slots are row-major from user side, left then right. Geometry extraction records include tessellation bounding tolerance; centers and straight-edge dimensions are used separately from those expanded bounds.

FAIL: at least the far-left final row slot is unreachable with a downward suction axis at the original low placement plane under the current numerical CAD model. See SLOT_ADJUSTMENT.md. Do not broaden hardware limits to conceal this.

Actual Gazebo views: evidence/view_isometric_gazebo.png, view_top_gazebo.png, view_front_gazebo.png, view_side_gazebo.png. Images are runtime evidence, not image-generation output.
