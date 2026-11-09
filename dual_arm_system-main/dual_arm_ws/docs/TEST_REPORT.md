# Evidence report — 2026-11-08

## 2026-11-09 Fresh GitHub clone check

Commit e345638 FAIL on a fresh clone (clone_build_check.log): `dual_arm_bringup` installed an empty `config` directory that git does not store. Removing that install entry gave 11 packages built, 33 cm530_hardware tests, coordinator tests and `start_sim.sh mock`, all PASS in a clone with the one-line fix. The earlier README flow check missed this because `tar` preserves empty directories.

## 2026-11-09 README flow check

The README steps were followed in a throwaway copy on the WSL Linux disk (readme_flow_check.log): copy 434 s and 578 MB, full build of 11 packages 300 s, then quick checks, 33 cm530_hardware tests, the mock-serial launch and `start_sim.sh mock`, all PASS. `install_jazzy.sh` was not rerun because the packages were already installed; its list was compared against the package dependency closure. All required ROS packages exist in the arm64 Jazzy repository, but an Orin build remains NOT_RUN.

## 2026-11-09 cm530_hardware on mock serial

Commands ran in WSL Ubuntu. The mock compiles the unmodified v17 firmware parser and AX-12 driver; only the motor bus and serial timing are simulated.

| Check | Result | Evidence / scope |
|---|---|---|
| `colcon build` of cm530_hardware, description, bringup | PASS | cm530_build.log; -Wall -Wextra -Wpedantic -Wconversion -Werror on project code |
| Unit tests (protocol, calibration, firmware-loop link and scheduler) | PASS | cm530_unit_tests.log; 33 tests |
| ASan + UBSan + LeakSanitizer, 20 repetitions | PASS | cm530_sanitizers.log |
| TSan, 20 repetitions | PASS | cm530_sanitizers.log; first attempt aborted at startup on the WSL mapping limit, rerun with `setarch -R` |
| `scripts/test_cm530_mock.py` | PASS | cm530_mock_integration.json; read-only, refused calibration, null template, missing tolerances, trajectory and mid-run link loss |
| URDF plugin selection in mock, sim, hardware and default modes | PASS | xacro output only; Gazebo was not rerun after this URDF edit |
| Real UART, motors, loop rate, Orin | NOT_RUN | No board connected |
| c-review security workflow | NOT_RUN | Its Workflow tool is unavailable in this environment |

## 2026-11-09 raw-512 zero mapping

Commands ran in WSL Ubuntu from `dual_arm_ws`.

| Check | Result | Evidence / scope |
|---|---|---|
| `bash scripts/build.sh` | PASS | colcon_build.log, model_validation.log; 10 packages |
| Coordinator unit tests | PASS | coordinator_zero512_tests.log; 8 tests |
| `scripts/report_home_poses.py` | PASS | home_poses.json; FK and limits only, simulation-assumed mapping |
| `scripts/check_home_collision.py` | PASS | home_collision.json; discrete self/inter-arm boxes, world and path NOT_RUN |
| Two-object A->B->C regression | PASS | transfer_pipeline.json, zero512_pipeline.log; Gazebo ground truth, headless, 74 events. cap_5 and cap_6 ended within 0.01 mm XY of their C targets |
| Simulation shutdown | FAIL | zero512_launch.log; launch still running 45 s after SIGINT, Gazebo stopped by SIGTERM |
| Arm 1 j1 zero | BLOCKED | Needs motor 17 readings on the real arm; 192.3 is a simulation placeholder |
| Hardware calibration, directions, wrist clearance | NOT_RUN | Hardware remains disabled |

A first collision attempt failed before running because the test command overrode PYTHONPATH and hid rclpy; it was rerun without the override. `transfer_pipeline.json` holds `FAIL` until a run completes, so a mid-run read is not a result.

## 2026-11-09 revision results

- `model_revision_validation.json`: PASS, ten prescribed component transforms, unchanged motor axes/TCP, eight white pads flush at 14 mm.
- `coordinator_revision_tests.log`: eight tests PASS, including HOME missing/invalid inputs and signed conversion.
- `home_named_states_build.log`: two affected packages built successfully after HOME named-state generation.
- `solidworks_standalone.log`: corrected geometry SLDPRT saved and reopened, 342 solid bodies, warnings=0. Not proof of original parametric mates or portable imported SLDASM references.
- Old three-object pipeline PASS is historical. Revised geometry / assumed HOME / 50% transport workflow is still undergoing runtime validation. Latest startup readiness failure is recorded, not counted as motion success.

All below refers to earlier component measurements unless explicitly regenerated.

These results cover separate components and measurements. Overall complete-system acceptance is **NOT_RUN**; the latest simulation runtime probe is **FAIL** because Gazebo shutdown fails. No hardware calibration or physical acceptance is claimed.

| Check | Recorded result | Evidence / scope |
|---|---|---|
| CAD source-pose transforms | PASS | evidence/cad_pose_validation.json; exported source pose, not dynamics |
| Original vision tests | PASS | evidence/vision_tests.log; 99 tests |
| Calibration validity and held-out plane checker | PASS | evidence/calibration_suite.log; 10 tests; no physical calibration |
| Mock controllers, MoveIt planning and shutdown | PASS | evidence/mock_runtime.json |
| Simulation both_arms small joint execution and feedback | PASS | evidence/sim_runtime.json; not A→B→C |
| Simulation shutdown | FAIL | evidence/sim_launch.log; Gazebo escalates to SIGTERM |
| Three camera streams and four views | PASS | evidence/sim_runtime.json; fixed simulation camera mounts |
| Suction isolated attachment/release physics | PASS | evidence/suction_physics.json; simple fixture, not dual-arm transfer |
| Suction policy sanitizers | PASS | evidence/gripper_sanitizers.log; policy test only |
| Saved slot-proposal FK and limits | PASS | evidence/slot_proposal_verification.json; 744 discrete points across three candidates |
| Proposal conservative-box self-collision | FAIL | evidence/slot_self_collision.json; −Y 7 mm: 49/248 invalid; −Y 10 mm and +X 19 mm: each 62/248 invalid |
| Selected CAD mesh surface witnesses | NOT_RUN (full clearance acceptance) | evidence/slot_mesh_witnesses.json; six representative states checked, no surface crossings found; containment and whole trajectory not covered |
| Full eight-object transfer and placement | NOT_RUN | Runtime coordinator, scene and validated target pipeline incomplete |
| CM530 / ESP32 / real cameras | NOT_RUN | Hardware disabled |

Saved-image coordinate measurements in evidence/sim_coordinate_validation.json are not calibration authorization. CameraInfo-derived maximum XY errors were approximately 1.97 mm (environment, 4 caps), 1.16 mm (arm1, 8 caps), and 1.61 mm (arm2, 8 holes). Four-point fitted homographies with four unused points in the same static scene reduce maximum held-out errors to approximately 0.17 mm and 0.09 mm for arm1 and arm2. Independent acquisition, timestamp/TF provenance and the measured combined error budget are still missing, so operational calibration remains NOT_RUN.

Reproduce the slot checks in WSL from the workspace:

```bash
python3 scripts/verify_slot_proposals.py
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/check_slot_self_collision.py  # expected exit 1 for recorded collisions
python3 scripts/check_slot_mesh_witnesses.py prepare
.cad-venv/bin/python scripts/check_slot_mesh_witnesses.py inspect
```

The collision query launches an isolated mock graph and sends state-validity requests only. It uses installed URDF/SRDF and verifies installed URDF bytes match source. Evidence includes model and input hashes. Rebuild if source and installed model differ.
