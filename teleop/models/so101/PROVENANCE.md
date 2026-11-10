# SO-101 URDF provenance

- Source: [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)
- Commit: `7629d2ad9853d10fb903093a33ef6114099d97e5`
- Source model: `Simulation/SO101/so101_new_calib.urdf`
- License: Apache-2.0; see the unmodified `LICENSE` file in this directory.
- Only mesh files referenced by the URDF were copied. Relative `assets/` paths were preserved.
- The source checkout remains outside runtime and unchanged.

## Temporary local IK trial (2026-11-10)

The runtime URDF differs from the upstream file only in the Motor 2 (`shoulder_lift`) limit, now ±1.94 rad, and Motor 3 (`elbow_flex`) limit, now ±1.71 rad. The calibration JSON was not changed; `SO101Kinematics` still intersects these model limits with the calibrated motor ranges. This allows the measured starting pose to enter IK without the automatic inward correction that stalled in the live test. The original URDF hash below identifies the upstream source; the current local URDF SHA-256 is `dbc2c5a814310716702dac93e9808a735e34286b8d1ae248c9caf3d2360dac9a`. Reassess the temporary limits after validating the physical workspace and camera-to-base direction.

## Upstream copied files (SHA-256 at import)

- `so101_new_calib.urdf` — 16231 bytes — `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`
- `LICENSE` — 11357 bytes — `c71d239df91726fc519c6eb72d318ec65820627232b2f796219e87dcf35d0ab4`
- `assets/base_motor_holder_so101_v1.stl` — 1877084 bytes — `8cd2f241037ea377af1191fffe0dd9d9006beea6dcc48543660ed41647072424`
- `assets/base_so101_v2.stl` — 471584 bytes — `bb12b7026575e1f70ccc7240051f9d943553bf34e5128537de6cd86fae33924d`
- `assets/motor_holder_so101_base_v1.stl` — 1129384 bytes — `31242ae6fb59d8b15c66617b88ad8e9bded62d57c35d11c0c43a70d2f4caa95b`
- `assets/motor_holder_so101_wrist_v1.stl` — 1052184 bytes — `887f92e6013cb64ea3a1ab8675e92da1e0beacfd5e001f972523540545e08011`
- `assets/moving_jaw_so101_v1.stl` — 1413584 bytes — `785a9dded2f474bc1d869e0d3dae398a3dcd9c0c345640040472210d2861fa9d`
- `assets/rotation_pitch_so101_v1.stl` — 883684 bytes — `9be900cc2a2bf718102841ef82ef8d2873842427648092c8ed2ca1e2ef4ffa34`
- `assets/sts3215_03a_no_horn_v1.stl` — 865884 bytes — `75ef3781b752e4065891aea855e34dc161a38a549549cd0970cedd07eae6f887`
- `assets/sts3215_03a_v1.stl` — 954084 bytes — `a37c871fb502483ab96c256baf457d36f2e97afc9205313d9c5ab275ef941cd0`
- `assets/under_arm_so101_v1.stl` — 1975884 bytes — `d01d1f2de365651dcad9d6669e94ff87ff7652b5bb2d10752a66a456a86dbc71`
- `assets/upper_arm_so101_v1.stl` — 1303484 bytes — `475056e03a17e71919b82fd88ab9a0b898ab50164f2a7943652a6b2941bb2d4f`
- `assets/waveshare_mounting_plate_so101_v2.stl` — 62784 bytes — `e197e24005a07d01bbc06a8c42311664eaeda415bf859f68fa247884d0f1a6e9`
- `assets/wrist_roll_follower_so101_v1.stl` — 1439884 bytes — `4b17b410a12d64ec39554abc3e8054d8a97384b2dc4a8d95a5ecb2a93670f5f4`
- `assets/wrist_roll_pitch_so101_v2.stl` — 2699784 bytes — `6c7ec5525b4d8b9e397a30ab4bb0037156a5d5f38a4adf2c7d943d6c56eda5ae`
