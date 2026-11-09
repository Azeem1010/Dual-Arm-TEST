# Architecture

The authoritative specification is REQUIREMENTS.md. Build simulation first on Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic, then validate physical deployment independently.

Description owns the two four-joint chains and TCP frames. Gazebo owns contact physics, cameras and simulated suction. MoveIt owns collision-aware planning. ros2_control owns execution; controllers have disjoint joints. One CM530 hardware instance will own the physical serial port.

Upstream vision interfaces remain intact. The adapter owns calibration provenance, image freshness, frame checks and conversion. Services without timestamps require correlated timestamped perception evidence; a response alone cannot authorize motion. Home-specific homographies are valid only at their calibrated pose. Calibration thresholds must be derived from measured clearance, cup geometry and mechanical uncertainty, not defaults.

Coordinator owns the A/B/C state machine. B ownership is released only after confirmed withdrawal, never merely on a timeout. Slots are committed only after vision verifies placement. Gripper feedback and MoveIt attached collision objects must agree.

Simulation and hardware evidence are stored separately. No default matrix authorizes hardware.
