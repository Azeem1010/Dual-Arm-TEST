# Source audit — 2026-11-08

Exact commits: sources.lock.json. Four requested repositories cloned into third_party. Integration copies live in src; no firmware edits made.

| Source | Finding |
|---|---|
| PhantomX | ros2 branch targets Galactic/Fortress. Contains component meshes and reusable macros. Windows checkout stores the meshes symlink as text; integration copies its actual target directory. Retain four revolute joints, remove servo fingers, add reference suction TCP. |
| CAD | SolidWorks assembly plus STEP available. README describes fixed pose with approximately 35/55 degree horns. Extracted 264 components and motor cylinder axes; source-pose transforms checked against the export. Moving component assignments and axes still need physical review; inertias and ±1.5 rad limits are provisional. Reference PhantomX is not CAD acceptance. |
| CM530 | v17 / protocol 5. arm_config.h defines A=[90,567,712,825], C=[590,567,712,825]. Interface spec section 4.3 incorrectly says all 512; runtime GET_HOME is authoritative for the actual board. Commands contain arm names, not motor IDs. One outstanding command per port. Timeout cannot safely retry identical commands because there are no sequence IDs. |
| Vision | Original vision_pkg and vision_interfaces copied without interface changes. Intrinsics, homography, caps calibration and HSV tools exist. Calibration templates are not verified physical evidence. GetPickTarget and GetGridState service responses have no header. Topic headers describe publication time, not necessarily acquisition time; adapter needs image provenance. |

Actual services: /vision/arm1/get_pick_target (GetPickTarget), /vision/arm2/get_grid_state (GetGridState), /vision/arm2/verify_place (VerifyPlace), /vision/env/get_zone_state (GetZoneState). Definitions are preserved in src/vision_interfaces/srv.

Protocol source check: APP/src/bridge.c handles PING/PONG and VERSION,5; APP/inc/arm_config.h confirms HOME constants. No claim of full c-review workflow (bare-metal firmware is outside its scope).

CM530 offline recheck 2026-11-09 (evidence/cm530_offline_recheck.log; no board connected, third_party unchanged):

| Check | Status | Note |
|---|---|---|
| HEX/BIN against SHA256SUMS | PASS | CM530.elf absent, so tests/verify_firmware.py cannot run |
| HOME presets and ID map in CM530.bin | PASS | 90,567,712,825 at 0x300C; 590,567,712,825 at 0x3014 |
| Terminal self-test | PASS | 25 tests |
| LED host harness, plain and ASan/UBSan | PASS | Compiled with gcc 13 and -w |
| Bridge host harness, plain and ASan/UBSan | FAIL | 11 checks, all stale expectations of HOME 512 in tests/test_bridge.c; no sanitizer reports. Compiled with -w because gcc 13 rejects test_bridge.c:38 under -Werror |
| Board flashed with this image, UART, motors | NOT_RUN | Confirm with GET_HOME on the real board |

The firmware never writes Moving Speed (address 32). ROBOTIS documents value 0 as maximum rpm without speed control, so HOME and large AX steps need host interpolation and a measured register value before any motion. HOME_CALIBRATION_TEST.md states the intent as arm1 right 30 deg and arm2 left 30 deg, and no completed measurement record for raw 90/590 was found.
