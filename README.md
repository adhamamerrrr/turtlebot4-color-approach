# Color approach controller - offline reference

Newly implemented in October 2026. The earlier README claimed recovered team
code, but no such source existed in the repository when this update was made.
This code is a new reference implementation, not recovered graduation code.

## Working functionality

- Reads validated ASCII RGB PPM (P3) images without third-party dependencies.
- Converts pixels to HSV and detects red across the hue wraparound.
- Finds the largest 4-connected component and rejects small noise components.
- Converts normalized image centroid/area into bounded approach/align decisions.
- Stops on missing targets, stale camera age, invalid sensor data, nearby obstacles
  or sufficient target image area. Turning and forward motion are separated when
  the target is off-center.

Python 3.10+:

```bash
python controller.py sample.ppm --frame-age 0 --clearance 1
python -m unittest -v
```

Distances are meters, age is seconds, linear speed is meters/second and angular
speed is radians/second. `--clearance` and `--frame-age` are explicit caller inputs,
not automatically measured. The CLI prints JSON and **never sends robot commands**.

## Hardware integration is intentionally not claimed

This repository has no ROS2 node, camera driver or LiDAR connection. To integrate
with TurtleBot4, supply calibrated camera frames, timestamped clearance data and
an independently validated velocity gate. Do not wire these offline commands
directly to a physical robot. Image area is not metric distance, red objects can
be ambiguous, and this controller does not account for footprint, stopping
distance, clearance freshness or obstacles outside a supplied clearance value.
No physical-robot, Gazebo, SLAM or navigation test is claimed here.

Seven tests run entirely offline, including a real image-to-CLI smoke check.
`sample.ppm` is a newly authored 10x10 synthetic image, not a captured robot frame.

Related larger reconstruction: https://github.com/adhamamerrrr/autonomous-turtlebot-ai
