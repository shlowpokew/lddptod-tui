//! Everything the UI shows. The ROS, serial, system and demo threads write
//! into one `State` behind a mutex; the UI thread locks it once per frame.

use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crate::msgs;

pub type Shared = Arc<Mutex<State>>;

pub const RATE_WINDOW: Duration = Duration::from_secs(3);
pub const SECTORS: usize = 8;
const SECTOR_WIDTH: f64 = std::f64::consts::TAU / SECTORS as f64;
pub const HIST_BIN_M: f64 = 0.25;
pub const HIST_BINS: usize = 32;

pub fn clock() -> String {
    chrono::Local::now().format("%H:%M:%S%.3f").to_string()
}

/// Message counter and rate estimate for one topic.
#[derive(Default)]
pub struct TopicStat {
    pub count: u64,
    pub last: Option<Instant>,
    pub error: Option<String>,
    arrivals: VecDeque<Instant>,
}

impl TopicStat {
    pub fn hit(&mut self, now: Instant) {
        self.count += 1;
        self.last = Some(now);
        self.arrivals.push_back(now);
        while let Some(&first) = self.arrivals.front() {
            if now.duration_since(first) > RATE_WINDOW || self.arrivals.len() > 4000 {
                self.arrivals.pop_front();
            } else {
                break;
            }
        }
    }

    pub fn hz(&self, now: Instant) -> f64 {
        let (Some(&first), Some(&last)) = (self.arrivals.front(), self.arrivals.back()) else {
            return 0.0;
        };
        if self.arrivals.len() < 2 || now.duration_since(last) > RATE_WINDOW {
            return 0.0;
        }
        let span = last.duration_since(first).as_secs_f64();
        if span <= 0.0 {
            0.0
        } else {
            (self.arrivals.len() - 1) as f64 / span
        }
    }

    pub fn age(&self, now: Instant) -> Option<Duration> {
        self.last.map(|t| now.duration_since(t))
    }
}

/// Fixed-size log buffer. `total` keeps counting so scrolled views can tell
/// how many lines arrived since they last looked.
pub struct Ring<T> {
    pub items: VecDeque<T>,
    cap: usize,
    pub total: u64,
}

impl<T> Ring<T> {
    pub fn new(cap: usize) -> Self {
        Ring { items: VecDeque::with_capacity(cap.min(1024)), cap, total: 0 }
    }

    pub fn push(&mut self, item: T) {
        if self.items.len() == self.cap {
            self.items.pop_front();
        }
        self.items.push_back(item);
        self.total += 1;
    }
}

pub const DEBUG: u8 = 10;
pub const INFO: u8 = 20;
pub const WARN: u8 = 30;
pub const ERROR: u8 = 40;
pub const FATAL: u8 = 50;

#[derive(Clone)]
pub struct LogLine {
    pub time: String,
    pub level: u8,
    pub source: String,
    pub text: String,
}

/// One LiDAR revolution, reduced to what the UI draws.
#[derive(Clone)]
pub struct ScanView {
    pub at: Instant,
    pub frame: String,
    /// Valid returns in the robot frame (x forward, y left), metres.
    pub points: Vec<(f64, f64)>,
    pub beams: usize,
    pub valid: usize,
    pub range_min: f64,
    pub range_max: f64,
    pub angle_min: f64,
    pub angle_max: f64,
    pub angle_increment: f64,
    pub scan_time: f64,
    /// Closest return: (distance, bearing in the robot frame).
    pub closest: Option<(f64, f64)>,
    pub farthest: f64,
    pub mean: f64,
    /// Same reduction the Uno gets: nearest return per 45 degree sector,
    /// sector 0 ahead, counter-clockwise.
    pub sectors: [Option<f64>; SECTORS],
    pub histogram: Vec<u64>,
}

impl ScanView {
    /// `beams` yields (angle in the sensor frame, range) per beam.
    pub fn build(
        frame: String,
        beams: impl Iterator<Item = (f64, f64)>,
        yaw: f64,
        range_min: f64,
        range_max: f64,
    ) -> ScanView {
        let mut view = ScanView {
            at: Instant::now(),
            frame,
            points: Vec::new(),
            beams: 0,
            valid: 0,
            range_min,
            range_max,
            angle_min: f64::NAN,
            angle_max: f64::NAN,
            angle_increment: f64::NAN,
            scan_time: f64::NAN,
            closest: None,
            farthest: 0.0,
            mean: 0.0,
            sectors: [None; SECTORS],
            histogram: vec![0; HIST_BINS],
        };
        let mut sum = 0.0;
        for (angle, range) in beams {
            view.beams += 1;
            let ok = range.is_finite()
                && range > 0.0
                && range >= range_min
                && (range_max <= 0.0 || range <= range_max);
            if !ok {
                continue;
            }
            view.valid += 1;
            sum += range;
            let bearing = angle + yaw;
            view.points.push((range * bearing.cos(), range * bearing.sin()));
            if view.closest.is_none_or(|(d, _)| range < d) {
                view.closest = Some((range, wrap_angle(bearing)));
            }
            view.farthest = view.farthest.max(range);
            let sector = ((bearing + SECTOR_WIDTH / 2.0) / SECTOR_WIDTH).floor() as i64;
            let sector = sector.rem_euclid(SECTORS as i64) as usize;
            if view.sectors[sector].is_none_or(|d| range < d) {
                view.sectors[sector] = Some(range);
            }
            let bin = ((range / HIST_BIN_M) as usize).min(HIST_BINS - 1);
            view.histogram[bin] += 1;
        }
        if view.valid > 0 {
            view.mean = sum / view.valid as f64;
        }
        view
    }

    pub fn from_msg(scan: &msgs::LaserScan, yaw: f64) -> ScanView {
        let start = scan.angle_min as f64;
        let step = scan.angle_increment as f64;
        let mut view = ScanView::build(
            scan.header.frame_id.clone(),
            scan.ranges.iter().enumerate().map(|(i, r)| (start + i as f64 * step, *r as f64)),
            yaw,
            scan.range_min as f64,
            scan.range_max as f64,
        );
        view.angle_min = scan.angle_min as f64;
        view.angle_max = scan.angle_max as f64;
        view.angle_increment = scan.angle_increment as f64;
        view.scan_time = scan.scan_time as f64;
        view
    }
}

pub fn wrap_angle(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}

/// 2D pose (x, y, yaw).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Pose2 {
    pub x: f64,
    pub y: f64,
    pub yaw: f64,
}

impl Pose2 {
    /// `self` then `other`, i.e. parent->a composed with a->b gives parent->b.
    pub fn then(self, other: Pose2) -> Pose2 {
        let (s, c) = self.yaw.sin_cos();
        Pose2 {
            x: self.x + c * other.x - s * other.y,
            y: self.y + s * other.x + c * other.y,
            yaw: wrap_angle(self.yaw + other.yaw),
        }
    }

    pub fn apply(self, x: f64, y: f64) -> (f64, f64) {
        let p = self.then(Pose2 { x, y, yaw: 0.0 });
        (p.x, p.y)
    }
}

pub struct TfEntry {
    pub pose: Pose2,
    pub z: f64,
    pub is_static: bool,
    pub stat: TopicStat,
}

pub struct OdomView {
    pub frame: String,
    pub child: String,
    pub pose: Pose2,
    pub linear: f64,
    pub angular: f64,
}

/// Occupancy grid flattened to world-frame points once per map update.
pub struct MapView {
    pub at: Instant,
    pub frame: String,
    pub width: u32,
    pub height: u32,
    pub resolution: f64,
    pub origin: Pose2,
    pub occupied: Vec<(f64, f64)>,
    pub free: Vec<(f64, f64)>,
    pub unknown_cells: usize,
    pub bounds: [f64; 4],
}

impl MapView {
    pub fn from_msg(grid: &msgs::OccupancyGrid) -> MapView {
        let info = &grid.info;
        let res = info.resolution as f64;
        let origin = Pose2 {
            x: info.origin.position.x,
            y: info.origin.position.y,
            yaw: info.origin.orientation.yaw(),
        };
        let (w, h) = (info.width as usize, info.height as usize);
        // Keep at most ~60k free-space dots; occupied cells are always drawn.
        let free_cells = grid.data.iter().filter(|v| **v == 0).count();
        let stride = (free_cells / 60_000 + 1).max(1);
        let mut occupied = Vec::new();
        let mut free = Vec::new();
        let mut unknown_cells = 0;
        let mut free_seen = 0usize;
        let mut bounds = [f64::MAX, f64::MAX, f64::MIN, f64::MIN];
        for (index, value) in grid.data.iter().enumerate().take(w * h) {
            let (col, row) = (index % w, index / w);
            let cell = || origin.apply((col as f64 + 0.5) * res, (row as f64 + 0.5) * res);
            match *value {
                v if v < 0 => unknown_cells += 1,
                v if v >= 50 => {
                    let p = cell();
                    bounds = [bounds[0].min(p.0), bounds[1].min(p.1), bounds[2].max(p.0), bounds[3].max(p.1)];
                    occupied.push(p);
                }
                0 => {
                    free_seen += 1;
                    if free_seen % stride == 0 {
                        let p = cell();
                        bounds = [bounds[0].min(p.0), bounds[1].min(p.1), bounds[2].max(p.0), bounds[3].max(p.1)];
                        free.push(p);
                    }
                }
                _ => {}
            }
        }
        if bounds[0] > bounds[2] {
            bounds = [-1.0, -1.0, 1.0, 1.0];
        }
        MapView {
            at: Instant::now(),
            frame: grid.header.frame_id.clone(),
            width: info.width,
            height: info.height,
            resolution: res,
            origin,
            occupied,
            free,
            unknown_cells,
            bounds,
        }
    }
}

#[derive(Default, Clone)]
pub struct SysInfo {
    pub hostname: String,
    pub cpu_temp_c: Option<f64>,
    pub cpu_percent: Option<f64>,
    pub load: String,
    pub mem_used_mb: Option<(u64, u64)>,
    pub disk: Option<String>,
    pub uptime: Option<Duration>,
    pub throttled: Option<String>,
    pub devices: Vec<(String, Option<String>)>,
    pub container: Option<String>,
    pub docker_note: Option<String>,
    pub git: Option<String>,
}

/// A topic seen on the DDS graph (ours or not).
#[derive(Clone)]
pub struct GraphTopic {
    pub name: String,
    pub type_name: String,
    pub publishers: usize,
    pub subscribers: usize,
}

pub struct State {
    pub started: Instant,
    pub domain: u16,
    pub ros_status: String,
    pub lidar_yaw: f64,
    pub topics: BTreeMap<String, TopicStat>,
    pub graph: Vec<GraphTopic>,

    pub scans: BTreeMap<String, ScanView>,
    pub odom: Option<OdomView>,
    pub wheel_velocity: Option<Vec<f32>>,
    pub cmd_vel: Option<(f64, f64)>,
    pub mission: Option<String>,
    pub mission_from_uno: bool,
    pub safety_warning: Option<String>,
    pub flags: BTreeMap<String, bool>,
    pub telemetry: Option<String>,
    pub telemetry_kv: Vec<(String, String)>,
    pub human_target: Option<(f64, f64)>,
    pub sos_targets: Vec<(f64, f64)>,
    pub tf: BTreeMap<(String, String), TfEntry>,
    pub map: Option<MapView>,
    pub breadcrumbs: Vec<(f64, f64)>,
    /// Direct serial LD19: (rotation speed deg/s, bad CRC count).
    pub serial_lidar: Option<(u16, u64)>,

    pub rosout: Ring<LogLine>,
    pub docker: Ring<LogLine>,
    pub events: Ring<LogLine>,
    pub sys: SysInfo,
}

impl State {
    pub fn new(domain: u16, lidar_yaw: f64) -> State {
        State {
            started: Instant::now(),
            domain,
            ros_status: "starting".into(),
            lidar_yaw,
            topics: BTreeMap::new(),
            graph: Vec::new(),
            scans: BTreeMap::new(),
            odom: None,
            wheel_velocity: None,
            cmd_vel: None,
            mission: None,
            mission_from_uno: false,
            safety_warning: None,
            flags: BTreeMap::new(),
            telemetry: None,
            telemetry_kv: Vec::new(),
            human_target: None,
            sos_targets: Vec::new(),
            tf: BTreeMap::new(),
            map: None,
            breadcrumbs: Vec::new(),
            serial_lidar: None,
            rosout: Ring::new(3000),
            docker: Ring::new(3000),
            events: Ring::new(1000),
            sys: SysInfo::default(),
        }
    }

    pub fn hit(&mut self, topic: &str) {
        let now = Instant::now();
        self.topics.entry(topic.to_string()).or_default().hit(now);
    }

    pub fn topic_error(&mut self, topic: &str, error: String) {
        let stat = self.topics.entry(topic.to_string()).or_default();
        if stat.error.as_deref() != Some(error.as_str()) {
            stat.error = Some(error.clone());
            self.event(ERROR, topic, format!("decode/read error: {error}"));
        }
    }

    pub fn event(&mut self, level: u8, source: &str, text: impl Into<String>) {
        self.events.push(LogLine { time: clock(), level, source: source.to_string(), text: text.into() });
    }

    pub fn set_flag(&mut self, topic: &str, value: bool) {
        let old = self.flags.insert(topic.to_string(), value);
        if old != Some(value) {
            self.event(INFO, topic, if value { "true" } else { "false" });
        }
    }

    pub fn set_mission(&mut self, mission: String, from_uno: bool) {
        // /mission/state only fires on change, so a TUI started mid-mission
        // never hears it. The Uno echoes the mode in every status line; use
        // that until /mission/state itself speaks, then trust /mission/state.
        if from_uno && self.mission.is_some() && !self.mission_from_uno {
            return;
        }
        if self.mission.as_deref() != Some(mission.as_str()) {
            let source = if from_uno { "/arduino/telemetry" } else { "/mission/state" };
            self.event(INFO, source, format!("mission -> {mission}"));
        }
        self.mission = Some(mission);
        self.mission_from_uno = from_uno;
    }

    pub fn set_safety(&mut self, warning: String) {
        let level = if warning.is_empty() { INFO } else { WARN };
        let text = if warning.is_empty() { "cleared".to_string() } else { warning.clone() };
        self.event(level, "/safety/warning", text);
        self.safety_warning = if warning.is_empty() { None } else { Some(warning) };
    }

    pub fn set_telemetry(&mut self, line: String) {
        self.telemetry_kv = line
            .split_whitespace()
            .filter_map(|w| w.split_once('=').map(|(k, v)| (k.to_string(), v.to_string())))
            .collect();
        if let Some((_, mode)) = self.telemetry_kv.iter().find(|(k, _)| k == "mission") {
            let mission = match mode.as_str() {
                "idle" => "S0_IDLE",
                "follow" => "S1_FOLLOW_MAP",
                "backtrack" => "S2_BACKTRACK",
                "sos" => "S3_SOS",
                other => other,
            }
            .to_string();
            self.set_mission(mission, true);
        }
        if self.telemetry_kv.is_empty() {
            // Not a status line: boot banner, error text, etc.
            self.event(INFO, "/arduino/telemetry", line.clone());
        }
        self.telemetry = Some(line);
    }

    pub fn set_tf(&mut self, transforms: &[msgs::TransformStamped], is_static: bool) {
        let now = Instant::now();
        for t in transforms {
            let key = (t.header.frame_id.trim_start_matches('/').to_string(), t.child_frame_id.trim_start_matches('/').to_string());
            let pose = Pose2 {
                x: t.transform.translation.x,
                y: t.transform.translation.y,
                yaw: t.transform.rotation.yaw(),
            };
            let entry = self.tf.entry(key).or_insert_with(|| TfEntry {
                pose,
                z: 0.0,
                is_static,
                stat: TopicStat::default(),
            });
            entry.pose = pose;
            entry.z = t.transform.translation.z;
            entry.is_static = is_static;
            entry.stat.hit(now);
        }
    }

    pub fn tf_pose(&self, parent: &str, child: &str) -> Option<Pose2> {
        self.tf.get(&(parent.to_string(), child.to_string())).map(|e| e.pose)
    }

    /// Robot pose and the frame it is in: map if SLAM is publishing
    /// map->odom, otherwise odom, otherwise the raw /odom message.
    pub fn robot_pose(&self) -> Option<(Pose2, &'static str)> {
        let odom_base = self.tf_pose("odom", "base_link").or_else(|| self.odom.as_ref().map(|o| o.pose));
        match (self.tf_pose("map", "odom"), odom_base) {
            (Some(map_odom), Some(odom_base)) => Some((map_odom.then(odom_base), "map")),
            (None, Some(odom_base)) => Some((odom_base, "odom")),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sectors_match_uno_link() {
        // uno_link.sector_minimums: sector 0 centred ahead, CCW, yaw added.
        let beams = vec![(0.0, 1.0), (std::f64::consts::FRAC_PI_2, 2.0), (-std::f64::consts::FRAC_PI_2, 3.0), (std::f64::consts::PI, 0.5)];
        let view = ScanView::build("laser".into(), beams.into_iter(), 0.0, 0.1, 8.0);
        assert_eq!(view.sectors[0], Some(1.0));
        assert_eq!(view.sectors[2], Some(2.0));
        assert_eq!(view.sectors[6], Some(3.0));
        assert_eq!(view.sectors[4], Some(0.5));
        assert_eq!(view.closest.map(|c| c.0), Some(0.5));
        assert_eq!(view.valid, 4);
    }

    #[test]
    fn invalid_ranges_are_dropped() {
        let beams = vec![(0.0, f64::NAN), (0.1, 0.05), (0.2, 9.0), (0.3, 1.0)];
        let view = ScanView::build("laser".into(), beams.into_iter(), 0.0, 0.15, 8.0);
        assert_eq!((view.beams, view.valid), (4, 1));
    }

    #[test]
    fn pose_composition() {
        let a = Pose2 { x: 1.0, y: 0.0, yaw: std::f64::consts::FRAC_PI_2 };
        let b = Pose2 { x: 1.0, y: 0.0, yaw: 0.0 };
        let c = a.then(b);
        assert!((c.x - 1.0).abs() < 1e-9 && (c.y - 1.0).abs() < 1e-9);
    }

    #[test]
    fn telemetry_sets_mission() {
        let mut state = State::new(42, 0.0);
        state.set_telemetry("st=driving mission=follow l=150 r=130 ahead=212 near=88 bad=0".into());
        assert_eq!(state.mission.as_deref(), Some("S1_FOLLOW_MAP"));
        assert_eq!(state.telemetry_kv.len(), 7);
    }
}
