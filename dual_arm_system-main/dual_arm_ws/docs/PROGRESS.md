# Development status — 2026-11-09

## Latest result: cm530_hardware on mock serial (supersedes the Hardware row below)

- New package src/cm530_hardware: protocol-5 parser, single-owner link, background scheduler, strict calibration and CM530SystemInterface plugin. New hardware.launch.py and URDF mode:=hardware.
- Tested against the unmodified v17 bridge.c, ax12.c and dynamixel.c with only the AX-12 bus simulated, including over a pseudo-terminal. 33 unit tests PASS; ASan/UBSan and TSan 20 repetitions PASS; five-phase launch integration PASS. Real UART timing, motors and the Orin remain NOT_RUN.
- Read-only by default. Motion is refused without measured calibration, verified Moving Speed, max_step_raw and a controllers_file with measured goal tolerances. Hardware starts inactive even then.
- Next hardware steps: fill calibration.yaml from real measurements (Arm 1 j1 still BLOCKED), measure the achievable loop rate, then a supervised first motion with an independent emergency stop. c-review was not run because its Workflow tool is unavailable here.

## Latest result: raw-512 zero mapping (supersedes the HOME and regression bullets below)

- User stated the physical zero is all four motors at raw 512 with the arm straight. Read as vertical, CAD q=(0,-35,-55,0) deg, so runtime.json raw_at_cad_zero j2..j4 = 631.35/699.55/512. Directions are inferred. Mapping stays simulation_assumed; hardware_use_allowed=false.
- Arm 1 j1 is UNRESOLVED: raw 90 is 123.75 deg from raw 512, yet the user reports home_A faces zone A and raw 512 faces +Y. raw_at_cad_zero 192.3 is a simulation placeholder (-30 deg). Needs motor 17 readings; see HOME_POSES.md.
- Wrist upper limit widened from 1.5 to 1.65 rad in simulation only, with user approval, so raw 825 fits. Physical clearance NOT_RUN.
- Build PASS, 8 coordinator tests PASS, HOME FK/limits PASS, HOME self/inter-arm collision PASS (home_collision.json; arm model only, world and path NOT_RUN).
- Two-object A->B->C regression PASS with revised geometry and this HOME (transfer_pipeline.json, zero512_pipeline.log): Gazebo ground truth, headless. Full eight-object, vision-driven, and PlanningScene acceptance remain NOT_RUN.
- Gazebo shutdown still FAIL: launch did not exit within 45 s of SIGINT and Gazebo needed SIGTERM.

## Current revision (supersedes historical rows below)

- User approved correcting the reversed camera support and filling C slots flush. Ten terminal components corrected, axes/TCP unchanged, 264 visuals retained, eight 25 mm pads top z=14 mm. Geometry checks PASS.
- Corrected STEP and standalone SolidWorks 2019 multi-body SLDPRT exported; SLDPRT reopened with 342 solid bodies and no warnings. Original newer-version native feature-tree assembly cannot be edited by installed SolidWorks 2019. Intermediate imported SLDASM and PackAndGo directory have unresolved portability concerns; do not deliver them alone.
- User approved simulation-assumed HOME conversion. Firmware counts preserved; simulated home_A/home_C map to base −/+30°, shoulder −10°, elbow 40°, wrist 60°. Named states built. Hardware calibration NOT_RUN, hardware use forbidden.
- Runtime scales now transport .50 / contact .25 / acceleration .25. Windows/WSL and Orin README and launch wrappers provided; Orin actual build/runtime NOT_RUN.
- Historical three-object overlapping pickup/transfer demonstration completed PASS before these geometry/HOME changes. New two-object regression is in progress; startup delay traced to ament package lookup on Windows-mounted storage. No claim of revised transfer acceptance yet.

See RESUME.md and latest evidence for ongoing results.

The complete requirements are **not yet met**. Hardware remains disabled. This ledger distinguishes recorded component evidence from end-to-end acceptance.

User decision: preserve original CAD and measure/propose the smallest platform/slot adjustment. See [SLOT_ADJUSTMENT.md](SLOT_ADJUSTMENT.md). No candidate has been applied to runtime geometry.

| Work | Current evidence and limitation |
|---|---|
| Source integration | Four requested repositories pinned in sources.lock.json; integration copies under src. |
| CAD | 264 exported components; source-pose transform check recorded PASS. Axis assignments, real joint zeros, mass/inertia and limits need physical validation. |
| ROS / MoveIt | Packages built in WSL Ubuntu 24.04 / Jazzy. Recorded mock planning and shutdown PASS. |
| Gazebo | Recorded both_arms small joint Plan & Execute and feedback PASS, three camera streams and four rendered views PASS. Overall runtime FAIL because Gazebo does not terminate cleanly. This is not a pick/place test. |
| Vision | Recorded upstream suite: 99 passed. Saved Gazebo image detection sees 8 caps in arm1 and 8 holes in arm2; environment sees 4 caps due occlusion. Live service freshness and calibrated TF authorization are not completed. |
| Calibration | Recorded 10 validity/plane-validation tests passed. Static-scene held-out homography errors measured, but all operational and hardware calibration validity remains NOT_RUN. |
| Suction | Recorded isolated Gazebo fixed-joint attachment/release physics test PASS, policy ASan/UBSan test PASS. Runtime coordinator and PlanningScene attachment synchronization not completed. |
| C reach | Original layout fails downward approach checks. −Y 7 mm and 10 mm candidates each rechecked at 248 points: discrete IK/FK PASS, but MoveIt conservative-box self-collision FAIL (49/248 and 62/248). Six selected CAD surface witnesses show no crossings; not sufficient to clear collision rejection. No world-collision or hardware acceptance. |
| A→B→C | NOT_RUN. Coordinator has preliminary zone/slot helpers and CAD IK; no complete runtime sequence. |
| Hardware | Protocol source audited. Single-port ros2_control backend, ESP32 integration and Orin deployment acceptance not completed. |

Next engineering work:

1. Resolve Gazebo shutdown failure without suppressing the failure result.
2. Validate a more accurate arm collision model against source CAD, resolve link_2/link_4 box rejection, then add a complete collision scene and validate approach/withdrawal for original reachable targets and separately named proposed layouts.
3. Acquire independent calibration observations; integrate image acquisition timestamps, TF provenance and freshness gates. Current homographies are measurements, not motion authorization.
4. Implement runtime gripper/PlanningScene synchronization, B-zone ownership and verified eight-slot coordinator. Original geometry cannot meet all-eight-slot reach acceptance as currently modelled.
5. Complete the single CM530 backend and failure tests from actual v17 protocol, ESP32 interface and Orin deployment instructions.

Recorded artifacts are under docs/evidence. Re-run affected checks after changing model, launch, world or calibration. Raw logs may describe earlier intermediate states; the JSON filenames cited here hold the latest recorded result of their respective scripts.
