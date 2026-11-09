# Implementation sequence

Spec: REQUIREMENTS.md. Execute in this workspace as requested.

- [ ] Audit four pinned sources and install Jazzy toolchain; record package versions and actual interface fields.
- [ ] Extract CAD component transforms and motor axes; retain upstream PhantomX kinematic reference. Test unique eight movable joints, connected world root, meshes and inertials.
- [ ] Build physical workstation, board, holes, caps and camera sensors. Check rendered views against CAD; save screenshots.
- [ ] Build ros2_control and MoveIt configurations. Test disjoint controllers, feedback and feasible four-DOF planning.
- [ ] Integrate original vision packages and Gazebo topics; run upstream tests and live image tests.
- [ ] Extend calibration validity and measurement reports; test invalid/missing/expired/moved-camera rejection and independent ground truth errors.
- [ ] Integrate fresh targets and TF2 into planning; reject stale data and invalid transformations.
- [ ] Implement physical suction, B ownership and sequential slots; run eight-object A-to-B-to-C test with placement verification.
- [ ] Audit v17 protocol, implement single-port backend with mock serial failure tests, then ESP32 interface.
- [ ] Document Orin installation and independent hardware acceptance. Mark unavailable physical tests NOT_RUN.

Review focus: stale image with successful service response; simulation evidence presented as hardware; four-DOF unreachable orientation; timeout releasing occupied B; ACK mistaken for position feedback.
