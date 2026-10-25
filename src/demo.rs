//! `--demo`: fake rover data, so the UI can be checked without a robot.

use std::thread;
use std::time::{Duration, Instant};

use crate::msgs;
use crate::state::{LogLine, MapView, OdomView, Pose2, ScanView, Shared, State, clock, INFO, WARN};

/// A 6 x 4 m room with a pillar, seen from `pose`, plus a person.
fn raycast(pose: Pose2, person: (f64, f64), beams: usize) -> Vec<(f64, f64)> {
    let mut out = Vec::with_capacity(beams);
    for i in 0..beams {
        let rel = -std::f64::consts::PI + i as f64 * std::f64::consts::TAU / beams as f64;
        let a = pose.yaw + rel;
        let (dx, dy) = (a.cos(), a.sin());
        let mut best = f64::INFINITY;
        // Walls at x = -3, 3 and y = -2, 2.
        for (wall, along_x) in [(-3.0, true), (3.0, true), (-2.0, false), (2.0, false)] {
            let (p, d) = if along_x { (pose.x, dx) } else { (pose.y, dy) };
            if d.abs() > 1e-9 {
                let t = (wall - p) / d;
                if t > 0.0 {
                    best = best.min(t);
                }
            }
        }
        for (cx, cy, r) in [(1.5, 0.8, 0.25), (person.0, person.1, 0.2)] {
            let (ox, oy) = (pose.x - cx, pose.y - cy);
            let b = ox * dx + oy * dy;
            let c = ox * ox + oy * oy - r * r;
            let disc = b * b - c;
            if disc >= 0.0 {
                let t = -b - disc.sqrt();
                if t > 0.0 {
                    best = best.min(t);
                }
            }
        }
        // A few dropouts like a real sensor.
        let range = if (i * 7919) % 53 == 0 { f64::NAN } else { best };
        out.push((rel, range));
    }
    out
}

fn demo_map() -> msgs::OccupancyGrid {
    let (w, h, res) = (140u32, 100u32, 0.05f32);
    let mut data = vec![-1i8; (w * h) as usize];
    for row in 0..h {
        for col in 0..w {
            let x = -3.5 + (col as f64 + 0.5) * res as f64;
            let y = -2.5 + (row as f64 + 0.5) * res as f64;
            let wall = (x.abs() - 3.0).abs() < 0.05 && y.abs() <= 2.0 || (y.abs() - 2.0).abs() < 0.05 && x.abs() <= 3.0;
            let pillar = ((x - 1.5).powi(2) + (y - 0.8).powi(2)).sqrt();
            let v = if wall || (pillar - 0.25).abs() < 0.04 {
                100
            } else if x.abs() < 3.0 && y.abs() < 2.0 && pillar > 0.25 {
                0
            } else {
                -1
            };
            data[(row * w + col) as usize] = v;
        }
    }
    msgs::OccupancyGrid {
        header: msgs::Header { stamp: msgs::Time::default(), frame_id: "map".into() },
        info: msgs::MapMetaData {
            map_load_time: msgs::Time::default(),
            resolution: res,
            width: w,
            height: h,
            origin: msgs::Pose {
                position: msgs::Vector3 { x: -3.5, y: -2.5, z: 0.0 },
                orientation: msgs::Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            },
        },
        data,
    }
}

/// One step of fake data at time `t` seconds.
pub fn step(st: &mut State, t: f64) {
    let pose = Pose2 { x: 1.6 * (t * 0.2).cos() - 0.4, y: 1.1 * (t * 0.2).sin(), yaw: t * 0.2 + std::f64::consts::FRAC_PI_2 };
    let person = (pose.x + 1.0 * pose.yaw.cos(), pose.y + 1.0 * pose.yaw.sin());
    let raw = raycast(pose, person, 456);
    st.scans.insert("/scan/raw".into(), ScanView::build("laser".into(), raw.iter().copied(), 0.0, 0.02, 12.0));
    st.hit("/scan/raw");
    st.scans.insert("/scan".into(), ScanView::build("laser".into(), raw.into_iter(), 0.0, 0.15, 8.0));
    st.hit("/scan");

    let map_odom = Pose2 { x: 0.05, y: -0.03, yaw: 0.02 };
    let odom_base = Pose2 { x: pose.x - 0.05, y: pose.y + 0.03, yaw: pose.yaw - 0.02 };
    st.odom = Some(OdomView { frame: "odom".into(), child: "base_link".into(), pose: odom_base, linear: 0.33, angular: 0.2 });
    st.hit("/odom");
    st.wheel_velocity = Some(vec![0.30, 0.36]);
    st.hit("/wheel/velocity");
    st.cmd_vel = Some((0.33, 0.2));
    st.hit("/cmd_vel");
    let tf = |parent: &str, child: &str, p: Pose2| msgs::TransformStamped {
        header: msgs::Header { stamp: msgs::Time::default(), frame_id: parent.into() },
        child_frame_id: child.into(),
        transform: msgs::Transform {
            translation: msgs::Vector3 { x: p.x, y: p.y, z: 0.0 },
            rotation: msgs::Quaternion { x: 0.0, y: 0.0, z: (p.yaw / 2.0).sin(), w: (p.yaw / 2.0).cos() },
        },
    };
    st.set_tf(&[tf("odom", "base_link", odom_base), tf("map", "odom", map_odom)], false);
    st.hit("/tf");
    if !st.tf.keys().any(|(_, c)| c == "laser") {
        st.set_tf(&[tf("base_link", "laser", Pose2::default())], true);
        st.hit("/tf_static");
    }

    // Person ahead, in base_link.
    let rel = Pose2 { x: 0.0, y: 0.0, yaw: -pose.yaw }.apply(person.0 - pose.x, person.1 - pose.y);
    st.human_target = Some(rel);
    st.hit("/human_target");

    let phase = (t / 20.0) as u64 % 4;
    let mission = ["S0_IDLE", "S1_FOLLOW_MAP", "S2_BACKTRACK", "S3_SOS"][phase as usize];
    if st.mission.as_deref() != Some(mission) {
        st.set_mission(mission.to_string(), false);
        st.hit("/mission/state");
        st.set_flag("/follow/enable", phase == 1);
        st.set_flag("/breadcrumb/recording", phase == 1);
        st.set_flag("/backtrack/enable", phase == 2 || phase == 3);
        if phase == 2 {
            st.set_safety("LiDAR scan stale; stopping motors".into());
        } else {
            st.set_safety(String::new());
        }
    }
    let near = st.scans.get("/scan").and_then(|s| s.closest).map(|c| (c.0 * 100.0) as i64).unwrap_or(0);
    let mode = ["idle", "follow", "backtrack", "sos"][phase as usize];
    st.set_telemetry(format!("st=driving mission={mode} l=150 r=130 ahead=212 near={near} bad=0"));
    st.hit("/arduino/telemetry");

    if st.map.is_none() {
        st.map = Some(MapView::from_msg(&demo_map()));
        st.hit("/map");
        st.breadcrumbs = (0..40).map(|i| {
            let a = i as f64 * 0.12;
            (1.6 * a.cos() - 0.4, 1.1 * a.sin())
        }).collect();
    }
}

pub fn spawn(shared: Shared) {
    thread::Builder::new()
        .name("demo".into())
        .spawn(move || {
            let start = Instant::now();
            let mut n: u64 = 0;
            {
                let mut st = shared.lock().unwrap();
                st.ros_status = "DEMO MODE (fake data)".into();
                for (name, ty, p, s) in [
                    ("/scan", "sensor_msgs/msg/LaserScan", 1, 5),
                    ("/scan/raw", "sensor_msgs/msg/LaserScan", 1, 1),
                    ("/odom", "nav_msgs/msg/Odometry", 1, 2),
                    ("/cmd_vel", "geometry_msgs/msg/Twist", 3, 1),
                    ("/rosout", "rcl_interfaces/msg/Log", 14, 1),
                    ("/map", "nav_msgs/msg/OccupancyGrid", 1, 0),
                ] {
                    st.graph.push(crate::state::GraphTopic { name: name.into(), type_name: ty.into(), publishers: p, subscribers: s });
                }
            }
            loop {
                let t = start.elapsed().as_secs_f64();
                let mut st = shared.lock().unwrap();
                step(&mut st, t);
                if n % 25 == 0 {
                    let level = if n % 100 == 0 { WARN } else { INFO };
                    st.rosout.push(LogLine { time: clock(), level, source: "human_tracker".into(), text: format!("demo log line {n}") });
                    st.hit("/rosout");
                    st.docker.push(LogLine { time: clock(), level: INFO, source: String::new(), text: format!("[python3-4] [INFO] [demo]: tick {n}") });
                }
                drop(st);
                n += 1;
                thread::sleep(Duration::from_millis(100));
            }
        })
        .expect("spawn demo thread");
}
