# apriltag_msgs on Foxy

`april_localizer.py` imports `AprilTagDetectionArray` from `apriltag_msgs`,
which is not in the Foxy apt repositories - only `ros-foxy-apriltag`, the C
detector library, is packaged. Without the messages the localiser starts,
logs `apriltag_msgs is not installed; AprilTag localization cannot run`, and
then silently never subscribes to detections.

## Getting it

```bash
cd <workspace>/src
git clone --depth 1 https://github.com/christianrauch/apriltag_msgs.git
cd apriltag_msgs
git apply <this dir>/0001-cmake-minimum-for-focal.patch
cd <workspace>
colcon build --packages-select apriltag_msgs
```

## What the patch changes

Upstream declares `cmake_minimum_required(VERSION 3.22)`. Ubuntu 20.04 ships
CMake 3.16.3, so the configure step fails immediately with an unhelpful
`exited with code 1` and no other output from colcon. Nothing in the package
actually uses a CMake 3.22 feature - it is three message definitions - so the
minimum is lowered to 3.8, which is what Foxy's own message packages declare.

Expect the same on the Unitree board: it is also Ubuntu 20.04, so the patch is
needed there too.

## What is in the messages

`AprilTagDetection` carries `family`, `id`, `hamming`, `goodness`,
`decision_margin`, `centre`, `corners` (four `Point`s in pixel coordinates) and
`homography`. It carries **no pose**: `april_localizer` recovers the tag pose
itself with `cv2.solvePnP` from the four corners plus `CameraInfo` and the
configured `tag_size`. A detector publishing this type therefore only has to
fill `id` and `corners` correctly - see `go2_control/apriltag_detect.py`.

Corner order matters and is not checked anywhere. `april_localizer` builds its
object points as top-left, top-right, bottom-right, bottom-left with y up,
which is the order `cv2.aruco.detectMarkers` returns. Publishing them in a
different order yields a pose that is plausible and wrong.
