# GLIM barometer extension

This production GLIM extension consumes `sensor_msgs/msg/FluidPressure` and
inserts a robust world-height factor for each active fixed-lag `Pose3` state.
It is deliberately a relative constraint: pressure is not treated as a precise
geodetic altitude.

The first valid pressure establishes a session reference for local odometry.
The global backend pairs its first submap-origin pose with pressure at that same
time and uses a separate offset for a new recording. Attached sessions constrain
height along their gravity axis, rotated into the reference map. This avoids accumulating incremental
pressure changes as a random walk. No sea-level pressure or field reference
pressure is configured. If `FluidPressure.variance` is
zero, negative, or non-finite, the extension uses the temporary `1 m`
per-sample vertical standard deviation. That uncertainty must be calibrated on
a static recording before operational use.

The first version assumes that the barometer and IMU have the same vertical
origin.  It does not estimate barometer bias or a barometer-to-IMU lever arm.

The USB device has no synchronized clock. Its ROS bridge stamps every
published `FluidPressure` message with the ROS reception time. A zero message
stamp is also treated as reception time by this extension. This makes the
factor receive-time aligned only; delayed USB transport is therefore a known
measurement-timing error and must be monitored before use on fast motion.

## USB bridge

The [lidar-lab](https://github.com/vratum/lidar-lab) project's
`scripts/barometer_bridge.py` accepts the documented text field `NNN dPa`,
such as `sensor: [2572275] -> 299 dC, 1002994 dPa`. It publishes
`100299.4 Pa` to `/barometer/pressure`; the device counter is intentionally
not used as a ROS timestamp. The connected RT T210C console was verified at
`921600` baud and inserts ANSI/prompt characters into its output; the bridge
removes them before extracting the `dPa` value. It still requires the actual
USB device path:

Run this command from the lidar-lab checkout:

```bash
source /opt/ros/jazzy/setup.bash
python3 scripts/barometer_bridge.py --ros-args \
  -p device:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5B14060368-if00
```

The bridge does not require `pyserial` and publishes `variance = 0.0` because
the documentation supplies no measurement variance. The GLIM extension then
uses its conservative fallback uncertainty.

The lidar-lab Mapping and GLIM UI actions start this bridge automatically
before GLIM and stop it when the mapping session ends.

## Build

```bash
colcon build --packages-select glim_ros
colcon test --packages-select glim_ros --ctest-args -R barometer_model --output-on-failure
```

The plugin is part of the regular `glim_ros` build and is installed into that
package's `lib` directory. From the lidar-lab checkout,
`bash scripts/build_glim_source.sh` builds both pinned GLIM forks and runs the
assertions-enabled barometer CTest. The height factor and archive registration live in
libglim so saved factors can be loaded without the sensor plugin. The test covers
archive roundtrip, rekey and a tilted height axis. It does not establish field
barometer accuracy or remove pressure transport delay.

## Runtime activation

Add the built `libglim_barometer_extension.so` from the `glim_ros` package's `lib`
directory to the session copy of
`glim_ros.extension_modules`; do not modify the base GLIM configuration.  The
extension subscribes to `/barometer/pressure` by default.

The lidar-lab launch helpers do this only for one mapping session:

```bash
python scripts/run_all.py \
  --barometer-extension-library /absolute/path/libglim_barometer_extension.so
```

That mode also records `/barometer/pressure` in the raw bag. Barometer factors
are inserted into both supported global mapping backends, including
`libglobal_mapping_pose_graph.so`.

Parameters:

- `barometer_topic` — `FluidPressure` topic.
- `barometer_default_height_stddev_m` — fallback per-sample uncertainty, default `1 m`.
- `barometer_queue_capacity` — maximum timestamp-ordered pending sample count.
