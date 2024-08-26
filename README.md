# Fiducials

Portfolio Post: www.cdiorio.dev/projects/camera-calibration/#fiducial-analysis

## Pi E-ink

This is a subproject that creates a server executable to be run on the raspberry pi that powers the e-ink display and a client
library.

## Comparator

Produces two executables:
- calibrate_extrinsics: 
  - Requires: 
    - the vicon syste,
    - eink display (displaying a calibration chessboard with vicon markers mounted to it) 
    - an intel realsense with attached vicon markers, mrcal rich and lean models
  - Uses the vicon system to take images and calibrate the T_hand_eye and T_mount_fiducial transforms

- take_measurements: 
  - the vicon system,
    - eink display (with attached markers and connected to computer) 
    - an intel realsense with attached vicon markers, mrcal rich and lean models
  - Measures the tags and reports their precision.

Notes to Developers:
- 
If a future MSR student is wanting to use this, reach out to me and I can direct you towards what you should take and what you should completely ignore.

While calibrate extrinsics attempts to perform the "dual" hand eye calibration problem present, the best ever performed was 0.250 millimeters of error and this is not good enough to measure tag accuracy and provide ground truth. This is because tags can achieve precision of tens of microns depending on the workspace size, tag size, and camera resolution.

Take measurements has effectively been used as a test script. There is a 
test runner class that allows for easy registering of test to run with the eink display and the main executable was always being reconfigured to run a variety of tests. So if you want to use it to run your own tests, it needs to be reprogrammed to run your specific tests. Taking into account the other note from above, the requirement on the vicon can be disabled if std::nullopt is passed for the vicon argument to the test runner. This allows you to still
run tests and gather relative to camera data and then fills in the vicon
specific fields with all zeros.
