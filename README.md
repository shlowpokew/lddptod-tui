# rover-tui

Terminal debugger for the [breadcrumbs rover](https://github.com/shlowpokew/breadcrumbs).
It's one static binary that runs on the Pi next to the ROS container and shows
everything in one place: LiDAR, odometry, TF, the SLAM map, breadcrumbs,
mission state, Uno telemetry, `/rosout`, docker logs and Pi health.

It joins the ROS 2 graph with a pure-Rust DDS client
([ros2-client](https://crates.io/crates/ros2-client)), so the Pi doesn't need
ROS, Rust or Python installed. It works because the container uses host
networking. It reads `ROS_DOMAIN_ID` from the breadcrumbs `.env` (default 42).

## Run it on the Pi

```sh
curl -fsSLO https://github.com/shlowpokew/lddptod-tui/releases/download/latest/rover-tui.sh
bash rover-tui.sh
```

The script downloads the prebuilt binary (about 5 MB) into
`~/.local/share/rover-tui/`. On later runs it only fetches a checksum, and
downloads the binary again only if it changed. `bash rover-tui.sh --offline`
skips the network. Every other argument goes to the binary:

| | |
| --- | --- |
| `bash rover-tui.sh` | normal: ROS topics + host checks + docker logs |
| `bash rover-tui.sh --lidar` | also read the LD19 straight off `/dev/lidar` (stop the container first: `sudo docker compose stop`) |
| `bash rover-tui.sh --no-ros --lidar` | sensor bring-up with no ROS at all |
| `bash rover-tui.sh --demo` | fake data, to check the UI |
| `bash rover-tui.sh --ros2 ~/breadcrumbs` | breadcrumbs checkout somewhere other than `~/etc/ros2` |
| `bash rover-tui.sh --help` | everything else |

Docker status and logs need `docker` access. That means being in the
`docker` group, or passwordless `sudo` (the default `pi` user has it).

## Tabs

`1`-`8` or `Tab` switch views, `q` quits.

1. **Overview**: mission state, safety warning, mode flags and buttons,
   odom/cmd_vel/wheel speeds, parsed Uno status line, a mini LiDAR plot,
   recent events and warnings, a per-topic health table (Hz, age,
   OK/SLOW/STALE/NO DATA), and Pi health: temperature, CPU, memory, disk,
   undervoltage/throttling, `/dev/lidar` and `/dev/uno`, container status,
   and the deployed git commit.
2. **LiDAR**: the full polar plot (`+`/`-` zoom, `s` cycles `/scan`,
   `/scan/raw` and the direct serial source), beam stats, the 45° sector
   minimums the Uno uses for its collision stop, and a range histogram.
3. **Map**: the slam_toolbox occupancy grid with the live scan overlaid in the
   map frame, the robot pose (map→odom→base_link), `maps/breadcrumbs.json`
   and the person target. Arrows or `hjkl` pan, `c` follows the robot again.
4. **Topics**: every topic on the DDS graph with type, pub/sub counts, rate
   and decode errors.
5. **TF**: the `map → odom → base_link → laser` chain check and every
   transform.
6. **ROS logs**: `/rosout` from all nodes. `l` sets the minimum level.
7. **Docker**: `docker logs -f ros2-mapping-1`. This works even when DDS
   shows nothing, for example when a node crashed or failed to start.
8. **Events**: mission changes, safety warnings, buttons, enables, manual Uno
   commands, all timestamped.

Scroll with `↑`/`↓`/`PgUp`/`PgDn`. `End` goes back to following the tail.

## If something's empty

- **Topics tab is empty but the Docker tab shows the nodes running**: DDS
  discovery isn't getting through. Check that `ROS_DOMAIN_ID` matches. Cyclone
  and this tool both need a network interface that supports multicast, so
  with no Wi-Fi at all, check `ip link`.
- **`/scan/raw` NO DATA**: the LD19 driver isn't publishing. Check
  `/dev/lidar` in the Pi panel. Then stop the container and run with
  `--no-ros --lidar`: that tests the sensor, cable and udev rule on their own,
  and shows motor speed and bad CRC counts.
- **TF MISSING map→odom**: slam_toolbox isn't localising yet. It needs
  `/scan` and odom→base_link first.

## Building

Every push to `main` builds `rover-tui-aarch64-linux` (static musl) on a
GitHub ARM runner, runs the tests, and uploads it to the rolling `latest`
release that `rover-tui.sh` pulls from. To build locally without installing
Rust, `bash build.sh` builds in a Rust Alpine container with Docker or
Podman. `bash build.sh pi@rover.local` also copies the result to the Pi.
