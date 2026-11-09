# Vision integration

The original packages are copied intact to src, pinned in docs/sources.lock.json.
`vision_sim.launch.py` launches the original env_vision_node, arm1_vision_node and arm2_vision_node against Gazebo image topics. It does not start USB devices.

Original service definitions and names remain unchanged. GetPickTarget response: success, message, rank, x/y/z. GetGridState response: occupied[8], place_x/y/z[8], all_full, success, message. VerifyPlace response: placed, order_ok, wrong_slots, success, message. These responses have no image acquisition timestamp.

Upstream frames use arm1_base / arm2_base; integration model uses arm1_base_link / arm2_base_link. A motion adapter must explicitly map and verify those frames with TF rather than merely relabel coordinates. This runtime target adapter is not yet implemented.

CalibrationStatus publishes diagnostic messages and fails closed with missing records. It is a monitor; it is not yet wired to a motion-executing coordinator. No automatic physical motion is currently exposed.
