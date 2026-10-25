//! Rendering. `View` is the UI-only state (tab, zoom, scroll); `State` is
//! the data, locked for the duration of one frame.

use std::time::{Duration, Instant};

use ratatui::layout::{Constraint, Layout, Rect};
use ratatui::style::{Color, Modifier, Style};
use ratatui::symbols::Marker;
use ratatui::text::{Line, Span};
use ratatui::widgets::canvas::{Canvas, Circle, Line as CLine, Points};
use ratatui::widgets::{Block, Cell, Paragraph, Row, Sparkline, Table, Tabs};
use ratatui::Frame;

use crate::state::{LogLine, Pose2, Ring, ScanView, State, DEBUG, ERROR, FATAL, HIST_BIN_M, INFO, SECTORS, WARN};

pub const TABS: [&str; 8] = ["Overview", "LiDAR", "Map", "Topics", "TF", "ROS logs", "Docker", "Events"];
const LOG_TABS: [usize; 3] = [5, 6, 7];

/// Topics the rover stack should be publishing: (topic, who, expected Hz).
pub const EXPECTED: &[(&str, &str, Option<f64>)] = &[
    ("/scan/raw", "LD19 driver", Some(10.0)),
    ("/scan", "lidar_scan_filter", Some(10.0)),
    ("/odom", "encoder_odometry", Some(50.0)),
    ("/wheel/velocity", "encoder_odometry", Some(50.0)),
    ("/tf", "odometry, slam", Some(50.0)),
    ("/tf_static", "static_transform_pub", None),
    ("/map", "slam_toolbox", None),
    ("/arduino/telemetry", "arduino_bridge/Uno", None),
    ("/cmd_vel", "controllers", None),
    ("/mission/state", "mission_manager", None),
    ("/safety/warning", "arduino_bridge", None),
    ("/human_target", "human_tracker", None),
    ("/sos/human_targets", "human_tracker", None),
    ("/rosout", "all nodes", None),
];

const SECTOR_NAMES: [&str; SECTORS] = ["ahead", "front-L", "left", "back-L", "behind", "back-R", "right", "front-R"];

const DIM: Color = Color::DarkGray;
const GRID: Color = Color::Indexed(238);

pub struct View {
    pub tab: usize,
    pub lidar_zoom: f64,
    pub scan_source: usize,
    pub map_zoom: f64,
    pub map_center: (f64, f64),
    pub map_follow: bool,
    pub scroll: [usize; TABS.len()],
    last_total: [u64; TABS.len()],
    pub min_level: u8,
}

impl Default for View {
    fn default() -> Self {
        View {
            tab: 0,
            lidar_zoom: 4.0,
            scan_source: 0,
            map_zoom: 4.0,
            map_center: (0.0, 0.0),
            map_follow: true,
            scroll: [0; TABS.len()],
            last_total: [0; TABS.len()],
            min_level: DEBUG,
        }
    }
}

pub enum Key {
    Char(char),
    Up,
    Down,
    Left,
    Right,
    PageUp,
    PageDown,
    Home,
    End,
    Tab,
    BackTab,
}

impl View {
    /// Returns false when the user asked to quit.
    pub fn on_key(&mut self, key: Key, st: &State) -> bool {
        let list_tab = self.tab >= 3;
        match key {
            Key::Char('q') => return false,
            Key::Char(c @ '1'..='8') => self.tab = c as usize - '1' as usize,
            Key::Tab => self.tab = (self.tab + 1) % TABS.len(),
            Key::BackTab => self.tab = (self.tab + TABS.len() - 1) % TABS.len(),
            Key::Char('+') | Key::Char('=') => self.zoom(1.0 / 1.4),
            Key::Char('-') | Key::Char('_') => self.zoom(1.4),
            Key::Char('s') => self.scan_source += 1,
            Key::Right | Key::Char('l') if self.tab == 2 => self.pan(st, 1.0, 0.0),
            Key::Char('l') => {
                self.min_level = match self.min_level {
                    DEBUG => INFO,
                    INFO => WARN,
                    WARN => ERROR,
                    _ => DEBUG,
                }
            }
            Key::Char('c') => {
                self.map_follow = true;
                self.scroll[self.tab] = 0;
            }
            Key::Up | Key::Char('k') if self.tab == 2 => self.pan(st, 0.0, 1.0),
            Key::Down | Key::Char('j') if self.tab == 2 => self.pan(st, 0.0, -1.0),
            Key::Left | Key::Char('h') if self.tab == 2 => self.pan(st, -1.0, 0.0),
            Key::Up | Key::Char('k') if list_tab => self.scroll_by(1),
            Key::Down | Key::Char('j') if list_tab => self.scroll_by(-1),
            Key::PageUp => self.scroll_by(20),
            Key::PageDown => self.scroll_by(-20),
            Key::Home => self.scroll[self.tab] = usize::MAX / 2,
            Key::End => self.scroll[self.tab] = 0,
            _ => {}
        }
        true
    }

    fn zoom(&mut self, factor: f64) {
        match self.tab {
            2 => self.map_zoom = (self.map_zoom * factor).clamp(0.5, 200.0),
            _ => self.lidar_zoom = (self.lidar_zoom * factor).clamp(0.5, 12.0),
        }
    }

    fn pan(&mut self, st: &State, dx: f64, dy: f64) {
        if self.map_follow {
            self.map_center = st.robot_pose().map(|(p, _)| (p.x, p.y)).unwrap_or(self.map_center);
            self.map_follow = false;
        }
        let step = self.map_zoom * 0.25;
        self.map_center.0 += dx * step;
        self.map_center.1 += dy * step;
    }

    fn scroll_by(&mut self, delta: i64) {
        let s = &mut self.scroll[self.tab];
        *s = if delta >= 0 { s.saturating_add(delta as usize) } else { s.saturating_sub((-delta) as usize) };
    }
}

// ---------------------------------------------------------------- helpers

fn age_text(age: Option<Duration>) -> String {
    match age {
        None => "never".into(),
        Some(d) if d.as_secs_f64() < 10.0 => format!("{:.1}s", d.as_secs_f64()),
        Some(d) if d.as_secs() < 120 => format!("{}s", d.as_secs()),
        Some(d) if d.as_secs() < 7200 => format!("{}m", d.as_secs() / 60),
        Some(d) => format!("{}h", d.as_secs() / 3600),
    }
}

fn level_style(level: u8) -> (Style, &'static str) {
    match level {
        l if l >= FATAL => (Style::new().fg(Color::White).bg(Color::Red).add_modifier(Modifier::BOLD), "FATAL"),
        l if l >= ERROR => (Style::new().fg(Color::Red).add_modifier(Modifier::BOLD), "ERROR"),
        l if l >= WARN => (Style::new().fg(Color::Yellow), "WARN "),
        l if l >= INFO => (Style::new(), "INFO "),
        _ => (Style::new().fg(DIM), "DEBUG"),
    }
}

fn mission_style(mission: &str) -> Style {
    let bg = match mission {
        m if m.starts_with("S1") => Color::Green,
        m if m.starts_with("S2") => Color::Yellow,
        m if m.starts_with("S3") => Color::Red,
        _ => Color::Gray,
    };
    let mut s = Style::new().fg(Color::Black).bg(bg).add_modifier(Modifier::BOLD);
    if mission.starts_with("S3") {
        s = s.add_modifier(Modifier::SLOW_BLINK);
    }
    s
}

fn distance_color(d: f64) -> Color {
    if d < 0.25 {
        Color::Red
    } else if d < 0.6 {
        Color::Yellow
    } else {
        Color::Green
    }
}

fn block(title: impl Into<String>) -> Block<'static> {
    Block::bordered().border_style(Style::new().fg(DIM)).title(Span::styled(
        format!(" {} ", title.into()),
        Style::new().fg(Color::Cyan).add_modifier(Modifier::BOLD),
    ))
}

fn kv(key: &str, value: impl Into<String>) -> Line<'static> {
    Line::from(vec![Span::styled(format!("{key:<10}"), Style::new().fg(DIM)), Span::raw(value.into())])
}

fn kv_styled(key: &str, value: impl Into<String>, style: Style) -> Line<'static> {
    Line::from(vec![Span::styled(format!("{key:<10}"), Style::new().fg(DIM)), Span::styled(value.into(), style)])
}

/// (status text, style) for a topic given what we expect of it.
fn topic_health(st: &State, topic: &str, expected: Option<f64>, now: Instant) -> (String, Style, f64, Option<Duration>) {
    let stat = st.topics.get(topic);
    let hz = stat.map(|s| s.hz(now)).unwrap_or(0.0);
    let age = stat.and_then(|s| s.age(now));
    let (text, style) = match (expected, age) {
        (_, _) if stat.and_then(|s| s.error.as_ref()).is_some() => ("DECODE ERR".into(), Style::new().fg(Color::Red).add_modifier(Modifier::BOLD)),
        (Some(_), None) => ("NO DATA".into(), Style::new().fg(Color::Red).add_modifier(Modifier::BOLD)),
        (Some(want), Some(a)) if a.as_secs_f64() > (3.0 / want).max(1.0) => ("STALE".into(), Style::new().fg(Color::Red).add_modifier(Modifier::BOLD)),
        (Some(want), Some(_)) if hz < want * 0.5 => ("SLOW".into(), Style::new().fg(Color::Yellow)),
        (Some(_), Some(_)) => ("ok".into(), Style::new().fg(Color::Green)),
        (None, None) => ("-".into(), Style::new().fg(DIM)),
        (None, Some(_)) => ("seen".into(), Style::new().fg(Color::Gray)),
    };
    (text, style, hz, age)
}

fn hz_text(hz: f64) -> String {
    if hz <= 0.0 {
        "-".into()
    } else if hz < 10.0 {
        format!("{hz:.1}")
    } else {
        format!("{hz:.0}")
    }
}

/// Canvas x/y bounds giving equal metres per dot in both directions.
fn square_bounds(area: Rect, cx: f64, cy: f64, half_height: f64) -> ([f64; 2], [f64; 2]) {
    let cols = area.width.saturating_sub(2).max(1) as f64;
    let rows = area.height.saturating_sub(2).max(1) as f64;
    let half_width = half_height * cols / (2.0 * rows);
    ([cx - half_width, cx + half_width], [cy - half_height, cy + half_height])
}

fn scan_keys(st: &State) -> Vec<String> {
    let mut keys: Vec<String> = ["/scan", "/scan/raw", "serial"]
        .iter()
        .filter(|k| st.scans.contains_key(**k))
        .map(|k| k.to_string())
        .collect();
    for k in st.scans.keys() {
        if !keys.contains(k) {
            keys.push(k.clone());
        }
    }
    keys
}

// ---------------------------------------------------------------- frame

pub fn draw(f: &mut Frame, st: &State, view: &mut View) {
    let [header, tabs, body, footer] = Layout::vertical([
        Constraint::Length(1),
        Constraint::Length(1),
        Constraint::Min(0),
        Constraint::Length(1),
    ])
    .areas(f.area());

    draw_header(f, header, st);
    let titles: Vec<String> = TABS.iter().enumerate().map(|(i, t)| format!("{} {}", i + 1, t)).collect();
    f.render_widget(
        Tabs::new(titles)
            .select(view.tab)
            .style(Style::new().fg(DIM))
            .highlight_style(Style::new().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD))
            .divider("|"),
        tabs,
    );

    match view.tab {
        0 => draw_overview(f, body, st, view),
        1 => draw_lidar(f, body, st, view),
        2 => draw_map(f, body, st, view),
        3 => draw_topics(f, body, st, view),
        4 => draw_tf(f, body, st, view),
        5 => {
            let min = view.min_level;
            let title = format!("/rosout  (level >= {}, l to change)", level_style(min).1.trim());
            draw_log(f, body, &st.rosout, view, 5, &title, |l| l.level >= min, true);
        }
        6 => draw_log(f, body, &st.docker, view, 6, "docker logs (container stdout/stderr)", |_| true, false),
        _ => draw_log(f, body, &st.events, view, 7, "events (state changes on the ROS graph)", |_| true, true),
    }

    let keys = match view.tab {
        1 => "q quit | 1-8 tabs | +/- zoom | s switch scan source",
        2 => "q quit | 1-8 tabs | +/- zoom | arrows/hjkl pan | c follow robot",
        3..=7 => "q quit | 1-8 tabs | up/down PgUp/PgDn scroll | End follow tail | l log level",
        _ => "q quit | 1-8 or Tab to switch views | LiDAR: +/- zoom",
    };
    f.render_widget(Paragraph::new(Span::styled(keys, Style::new().fg(DIM))), footer);
}

fn draw_header(f: &mut Frame, area: Rect, st: &State) {
    let mut spans = vec![
        Span::styled(" BREADCRUMBS ROVER ", Style::new().fg(Color::Black).bg(Color::Cyan).add_modifier(Modifier::BOLD)),
        Span::raw(" "),
    ];
    match &st.mission {
        Some(m) => spans.push(Span::styled(format!(" {m} "), mission_style(m))),
        None => spans.push(Span::styled(" mission ? ", Style::new().fg(Color::Black).bg(DIM))),
    }
    spans.push(Span::raw(" "));
    if let Some(w) = &st.safety_warning {
        spans.push(Span::styled(format!(" SAFETY: {w} "), Style::new().fg(Color::White).bg(Color::Red).add_modifier(Modifier::BOLD)));
        spans.push(Span::raw(" "));
    }
    let host = if st.sys.hostname.is_empty() { String::new() } else { format!("{} | ", st.sys.hostname) };
    spans.push(Span::styled(
        format!("{host}domain {} | {} | up {}", st.domain, st.ros_status, age_text(Some(st.started.elapsed()))),
        Style::new().fg(DIM),
    ));
    f.render_widget(Paragraph::new(Line::from(spans)), area);
}

// ---------------------------------------------------------------- overview

fn draw_overview(f: &mut Frame, area: Rect, st: &State, view: &View) {
    let [left, mid, right] = Layout::horizontal([
        Constraint::Length(36),
        Constraint::Min(30),
        Constraint::Length(48),
    ])
    .areas(area);

    // Left: mission, motion, Uno.
    let [mission_area, motion_area, uno_area] = Layout::vertical([
        Constraint::Length(9),
        Constraint::Length(9),
        Constraint::Min(4),
    ])
    .areas(left);

    let mut lines = vec![match &st.mission {
        Some(m) => Line::from(vec![
            Span::styled(format!(" {m} "), mission_style(m)),
            Span::styled(if st.mission_from_uno { "  (from Uno)" } else { "" }, Style::new().fg(DIM)),
        ]),
        None => Line::styled("unknown (no /mission/state yet)", Style::new().fg(DIM)),
    }];
    lines.push(match &st.safety_warning {
        Some(w) => kv_styled("safety", w.clone(), Style::new().fg(Color::Red).add_modifier(Modifier::BOLD)),
        None => kv_styled("safety", "clear", Style::new().fg(Color::Green)),
    });
    let flag = |topic: &str, label: &str| -> Span<'static> {
        match st.flags.get(topic) {
            Some(true) => Span::styled(format!("{label} "), Style::new().fg(Color::Black).bg(Color::Green)),
            Some(false) => Span::styled(format!("{label} "), Style::new().fg(DIM)),
            None => Span::styled(format!("{label}? "), Style::new().fg(GRID)),
        }
    };
    lines.push(Line::from(vec![flag("/follow/enable", "follow"), flag("/breadcrumb/recording", "rec"), flag("/backtrack/enable", "backtrack")]));
    lines.push(Line::from(vec![flag("/sos/seek_enable", "seek"), flag("/sos/human_nearby", "human<3m"), flag("/backtrack/complete", "returned")]));
    lines.push(Line::from(vec![flag("/hardware/button_a", "btnA"), flag("/hardware/button_b", "btnB"), flag("/hardware/sos", "SOS")]));
    lines.push(match st.human_target {
        Some((x, y)) => kv("person", format!("{:.2} m @ {:+.0} deg", x.hypot(y), y.atan2(x).to_degrees())),
        None => kv_styled("person", "none", Style::new().fg(DIM)),
    });
    lines.push(kv("sos tgts", format!("{}", st.sos_targets.len())));
    f.render_widget(Paragraph::new(lines).block(block("Mission")), mission_area);

    let mut lines = Vec::new();
    match &st.odom {
        Some(o) => {
            lines.push(kv("odom", format!("x {:+.2} y {:+.2}", o.pose.x, o.pose.y)));
            lines.push(kv("heading", format!("{:+.1} deg", o.pose.yaw.to_degrees())));
            lines.push(kv("speed", format!("{:+.2} m/s {:+.2} rad/s", o.linear, o.angular)));
        }
        None => lines.push(kv_styled("odom", "no /odom", Style::new().fg(Color::Red))),
    }
    match st.robot_pose() {
        Some((p, frame)) => lines.push(kv(&format!("in {frame}"), format!("x {:+.2} y {:+.2} {:+.0}deg", p.x, p.y, p.yaw.to_degrees()))),
        None => lines.push(kv_styled("pose", "no TF", Style::new().fg(DIM))),
    }
    match st.cmd_vel {
        Some((v, w)) => lines.push(kv("cmd_vel", format!("{v:+.2} m/s {w:+.2} rad/s"))),
        None => lines.push(kv_styled("cmd_vel", "none", Style::new().fg(DIM))),
    }
    match &st.wheel_velocity {
        Some(w) if w.len() >= 2 => lines.push(kv("wheels", format!("L {:+.2} R {:+.2} m/s", w[0], w[1]))),
        _ => lines.push(kv_styled("wheels", "no /wheel/velocity", Style::new().fg(DIM))),
    }
    let age = st.topics.get("/cmd_vel").and_then(|s| s.age(Instant::now()));
    lines.push(kv("cmd age", age_text(age)));
    f.render_widget(Paragraph::new(lines).block(block("Motion")), motion_area);

    let mut lines: Vec<Line> = st.telemetry_kv.iter().map(|(k, v)| {
        let style = match (k.as_str(), v.parse::<f64>()) {
            ("near", Ok(cm)) | ("ahead", Ok(cm)) => Style::new().fg(distance_color(cm / 100.0)),
            ("bad", Ok(n)) if n > 0.0 => Style::new().fg(Color::Red),
            _ => Style::new(),
        };
        kv_styled(k, v.clone(), style)
    }).collect();
    if lines.is_empty() {
        lines.push(Line::styled(st.telemetry.clone().unwrap_or_else(|| "no /arduino/telemetry".into()), Style::new().fg(DIM)));
    }
    let age = st.topics.get("/arduino/telemetry").and_then(|s| s.age(Instant::now()));
    f.render_widget(Paragraph::new(lines).block(block(format!("Uno ({})", age_text(age)))), uno_area);

    // Middle: scan + recent events.
    let [scan_area, events_area] = Layout::vertical([Constraint::Percentage(68), Constraint::Min(5)]).areas(mid);
    let keys = scan_keys(st);
    let key = keys.get(view.scan_source % keys.len().max(1)).cloned();
    draw_scan_canvas(f, scan_area, st, key.as_deref(), view.lidar_zoom);
    let height = events_area.height.saturating_sub(2) as usize;
    let mut recent: Vec<&LogLine> = st
        .events
        .items
        .iter()
        .chain(st.rosout.items.iter().filter(|l| l.level >= WARN))
        .collect();
    recent.sort_by(|a, b| a.time.cmp(&b.time));
    let lines: Vec<Line> = recent.iter().rev().take(height).rev().map(|l| log_line(l, true)).collect();
    f.render_widget(Paragraph::new(lines).block(block("Recent events + warnings")), events_area);

    // Right: topic health + system.
    let [health_area, sys_area] = Layout::vertical([Constraint::Length(EXPECTED.len() as u16 + 3), Constraint::Min(6)]).areas(right);
    let now = Instant::now();
    let rows: Vec<Row> = EXPECTED.iter().map(|(topic, _, want)| {
        let (status, style, hz, age) = topic_health(st, topic, *want, now);
        Row::new(vec![
            Cell::from(topic.to_string()),
            Cell::from(hz_text(hz)),
            Cell::from(age_text(age)),
            Cell::from(Span::styled(status, style)),
        ])
    }).collect();
    let table = Table::new(rows, [Constraint::Length(19), Constraint::Length(5), Constraint::Length(6), Constraint::Min(6)])
        .header(Row::new(vec!["topic", "Hz", "age", "status"]).style(Style::new().fg(DIM)))
        .block(block("Topic health"));
    f.render_widget(table, health_area);
    draw_system(f, sys_area, st);
}

fn draw_system(f: &mut Frame, area: Rect, st: &State) {
    let s = &st.sys;
    let mut lines = Vec::new();
    let temp = s.cpu_temp_c.map(|t| (format!("{t:.1} C"), if t > 80.0 { Color::Red } else if t > 70.0 { Color::Yellow } else { Color::Green }));
    match temp {
        Some((t, c)) => lines.push(kv_styled("cpu temp", t, Style::new().fg(c))),
        None => lines.push(kv_styled("cpu temp", "n/a", Style::new().fg(DIM))),
    }
    lines.push(kv("cpu", format!("{} | load {}", s.cpu_percent.map(|p| format!("{p:.0}%")).unwrap_or("-".into()), s.load)));
    if let Some((used, total)) = s.mem_used_mb {
        lines.push(kv("mem", format!("{used} / {total} MB")));
    }
    if let Some(d) = &s.disk {
        lines.push(kv("disk /", d.clone()));
    }
    if let Some(t) = &s.throttled {
        let style = if t == "ok" { Style::new().fg(Color::Green) } else { Style::new().fg(Color::Red).add_modifier(Modifier::BOLD) };
        lines.push(kv_styled("power", t.clone(), style));
    }
    for (dev, target) in &s.devices {
        let name = dev.trim_start_matches("/dev/");
        match target {
            Some(t) => lines.push(kv_styled(name, t.clone(), Style::new().fg(Color::Green))),
            None => lines.push(kv_styled(name, "MISSING", Style::new().fg(Color::Red).add_modifier(Modifier::BOLD))),
        }
    }
    match (&s.container, &s.docker_note) {
        (Some(c), _) => {
            let style = if c.starts_with("Up") { Style::new().fg(Color::Green) } else { Style::new().fg(Color::Red).add_modifier(Modifier::BOLD) };
            lines.push(kv_styled("container", c.clone(), style));
        }
        (None, Some(note)) => lines.push(kv_styled("container", note.clone(), Style::new().fg(Color::Yellow))),
        (None, None) => lines.push(kv_styled("container", "checking...", Style::new().fg(DIM))),
    }
    if let Some(g) = &s.git {
        lines.push(kv("ros2 repo", g.clone()));
    }
    if let Some(u) = s.uptime {
        lines.push(kv("pi uptime", age_text(Some(u))));
    }
    f.render_widget(Paragraph::new(lines).block(block("Pi")), area);
}

// ---------------------------------------------------------------- lidar

fn draw_scan_canvas(f: &mut Frame, area: Rect, st: &State, key: Option<&str>, zoom: f64) {
    let scan = key.and_then(|k| st.scans.get(k));
    let title = match (key, scan) {
        (Some(k), Some(s)) => format!("LiDAR {k}  {} pts  {}  zoom {zoom:.1} m", s.valid, age_text(Some(s.at.elapsed()))),
        _ => "LiDAR: no scan yet".into(),
    };
    let (xb, yb) = square_bounds(area, 0.0, 0.0, zoom);
    // Robot frame (x forward, y left) to screen (right, up): forward is up.
    let screen = |x: f64, y: f64| (-y, x);
    let mut near = Vec::new();
    let mut mid = Vec::new();
    let mut far = Vec::new();
    if let Some(s) = scan {
        for &(x, y) in &s.points {
            let d = x.hypot(y);
            let p = screen(x, y);
            if d < 0.3 {
                near.push(p);
            } else if d < 1.0 {
                mid.push(p);
            } else {
                far.push(p);
            }
        }
    }
    let closest = scan.and_then(|s| s.closest);
    let target = st.human_target;
    let sos: Vec<(f64, f64)> = st.sos_targets.iter().map(|&(x, y)| screen(x, y)).collect();
    let ring_step = if zoom > 6.0 { 2.0 } else if zoom > 2.0 { 1.0 } else { 0.5 };
    let reach = zoom * 2.5;
    let canvas = Canvas::default()
        .block(block(title))
        .marker(Marker::Braille)
        .x_bounds(xb)
        .y_bounds(yb)
        .paint(move |ctx| {
            let mut r = ring_step;
            while r <= reach {
                ctx.draw(&Circle { x: 0.0, y: 0.0, radius: r, color: GRID });
                r += ring_step;
            }
            for k in 0..SECTORS {
                let a = (k as f64 + 0.5) * std::f64::consts::TAU / SECTORS as f64;
                let (x2, y2) = screen(reach * a.cos(), reach * a.sin());
                let (x1, y1) = screen(0.3 * a.cos(), 0.3 * a.sin());
                ctx.draw(&CLine { x1, y1, x2, y2, color: GRID });
            }
            ctx.layer();
            ctx.draw(&Points { coords: &far, color: Color::Green });
            ctx.draw(&Points { coords: &mid, color: Color::Yellow });
            ctx.draw(&Points { coords: &near, color: Color::Red });
            // Robot: box and heading.
            let (hw, hl) = (0.12, 0.16);
            for (a, b) in [((-hw, -hl), (hw, -hl)), ((hw, -hl), (hw, hl)), ((hw, hl), (-hw, hl)), ((-hw, hl), (-hw, -hl))] {
                ctx.draw(&CLine { x1: a.0, y1: a.1, x2: b.0, y2: b.1, color: Color::Cyan });
            }
            ctx.draw(&CLine { x1: 0.0, y1: 0.0, x2: 0.0, y2: 0.4, color: Color::Cyan });
            if let Some((x, y)) = target {
                let (sx, sy) = screen(x, y);
                ctx.draw(&Circle { x: sx, y: sy, radius: 0.2, color: Color::Magenta });
                ctx.print(sx + 0.25, sy, Span::styled("person", Style::new().fg(Color::Magenta)));
            }
            for &(sx, sy) in &sos {
                ctx.draw(&Circle { x: sx, y: sy, radius: 0.25, color: Color::LightRed });
            }
            if let Some((d, bearing)) = closest {
                let (sx, sy) = screen(d * bearing.cos(), d * bearing.sin());
                ctx.print(sx, sy, Span::styled(format!("x {d:.2}m"), Style::new().fg(distance_color(d)).add_modifier(Modifier::BOLD)));
            }
            let mut r = ring_step;
            while r <= zoom {
                ctx.print(0.05, r, Span::styled(format!("{r}m"), Style::new().fg(DIM)));
                r += ring_step;
            }
        });
    f.render_widget(canvas, area);
}

fn draw_lidar(f: &mut Frame, area: Rect, st: &State, view: &View) {
    let [canvas_area, side] = Layout::horizontal([Constraint::Min(30), Constraint::Length(40)]).areas(area);
    let keys = scan_keys(st);
    let key = keys.get(view.scan_source % keys.len().max(1)).cloned();
    draw_scan_canvas(f, canvas_area, st, key.as_deref(), view.lidar_zoom);

    let [stats_area, sectors_area, hist_area] = Layout::vertical([
        Constraint::Length(16),
        Constraint::Length(SECTORS as u16 + 2),
        Constraint::Min(4),
    ])
    .areas(side);

    let scan: Option<&ScanView> = key.as_deref().and_then(|k| st.scans.get(k));
    let mut lines = vec![kv("sources", if keys.is_empty() { "none".into() } else { keys.join(", ") })];
    match (key.as_deref(), scan) {
        (Some(k), Some(s)) => {
            let now = Instant::now();
            let stat_key = if k == "serial" { crate::lidar::SERIAL_TOPIC } else { k };
            let hz = st.topics.get(stat_key).map(|t| t.hz(now)).unwrap_or(0.0);
            let pct = if s.beams > 0 { 100.0 * s.valid as f64 / s.beams as f64 } else { 0.0 };
            lines.push(kv("showing", format!("{k} (s to switch)")));
            lines.push(kv("frame", s.frame.clone()));
            lines.push(kv("rate", format!("{} Hz, age {}", hz_text(hz), age_text(Some(s.at.elapsed())))));
            lines.push(kv_styled("valid", format!("{}/{} beams ({pct:.0}%)", s.valid, s.beams), Style::new().fg(if pct < 50.0 { Color::Yellow } else { Color::Green })));
            lines.push(kv("limits", format!("{:.2} .. {:.2} m", s.range_min, s.range_max)));
            if s.angle_min.is_finite() {
                lines.push(kv("angles", format!("{:.1} .. {:.1} deg", s.angle_min.to_degrees(), s.angle_max.to_degrees())));
                lines.push(kv("step", format!("{:.3} deg", s.angle_increment.to_degrees())));
                lines.push(kv("scan_time", format!("{:.3} s", s.scan_time)));
            }
            match s.closest {
                Some((d, b)) => lines.push(kv_styled("closest", format!("{d:.2} m @ {:+.0} deg", b.to_degrees()), Style::new().fg(distance_color(d)).add_modifier(Modifier::BOLD))),
                None => lines.push(kv("closest", "-")),
            }
            lines.push(kv("mean/max", format!("{:.2} / {:.2} m", s.mean, s.farthest)));
            if k == "serial" {
                if let Some((speed, bad)) = st.serial_lidar {
                    lines.push(kv("motor", format!("{:.1} rev/s ({speed} deg/s)", speed as f64 / 360.0)));
                    lines.push(kv_styled("bad crc", bad.to_string(), Style::new().fg(if bad > 0 { Color::Yellow } else { Color::Green })));
                }
            }
        }
        _ => {
            lines.push(Line::styled("No scans received.", Style::new().fg(Color::Red)));
            lines.push(Line::styled("Check Topics tab and /dev/lidar.", Style::new().fg(DIM)));
            lines.push(Line::styled("Container down? run with --lidar", Style::new().fg(DIM)));
        }
    }
    f.render_widget(Paragraph::new(lines).block(block("Scan")), stats_area);

    let bar_width = sectors_area.width.saturating_sub(22) as f64;
    let lines: Vec<Line> = (0..SECTORS).map(|i| {
        let d = scan.and_then(|s| s.sectors[i]);
        let (bar, text, color) = match d {
            Some(d) => {
                let n = ((d.min(3.0) / 3.0) * bar_width).round().max(1.0) as usize;
                ("#".repeat(n), format!("{d:5.2}m"), distance_color(d))
            }
            None => (String::new(), "  none".into(), DIM),
        };
        Line::from(vec![
            Span::styled(format!("{:<8}", SECTOR_NAMES[i]), Style::new().fg(DIM)),
            Span::styled(text, Style::new().fg(color)),
            Span::raw(" "),
            Span::styled(bar, Style::new().fg(color)),
        ])
    }).collect();
    f.render_widget(Paragraph::new(lines).block(block("Nearest per 45 deg (what the Uno sees)")), sectors_area);

    let hist: Vec<u64> = scan.map(|s| s.histogram.clone()).unwrap_or_default();
    f.render_widget(
        Sparkline::default()
            .data(&hist)
            .style(Style::new().fg(Color::Cyan))
            .block(block(format!("Range histogram 0-{:.0} m", HIST_BIN_M * hist.len().max(1) as f64))),
        hist_area,
    );
}

// ---------------------------------------------------------------- map

fn draw_map(f: &mut Frame, area: Rect, st: &State, view: &mut View) {
    let [canvas_area, side] = Layout::horizontal([Constraint::Min(30), Constraint::Length(38)]).areas(area);
    let robot = st.robot_pose();
    if view.map_follow {
        if let Some((p, _)) = robot {
            view.map_center = (p.x, p.y);
        } else if let Some(m) = &st.map {
            view.map_center = ((m.bounds[0] + m.bounds[2]) / 2.0, (m.bounds[1] + m.bounds[3]) / 2.0);
        }
    }
    let (xb, yb) = square_bounds(canvas_area, view.map_center.0, view.map_center.1, view.map_zoom);
    let in_view = |p: &(f64, f64)| p.0 >= xb[0] && p.0 <= xb[1] && p.1 >= yb[0] && p.1 <= yb[1];
    let (free, occupied): (Vec<(f64, f64)>, Vec<(f64, f64)>) = match &st.map {
        Some(m) => (m.free.iter().copied().filter(in_view).collect(), m.occupied.iter().copied().filter(in_view).collect()),
        None => (Vec::new(), Vec::new()),
    };
    let scan_world: Vec<(f64, f64)> = match (robot, st.scans.get("/scan")) {
        (Some((pose, _)), Some(s)) => s.points.iter().map(|&(x, y)| pose.apply(x, y)).collect(),
        _ => Vec::new(),
    };
    let crumbs = st.breadcrumbs.clone();
    let target = match (robot, st.human_target) {
        (Some((pose, _)), Some((x, y))) => Some(pose.apply(x, y)),
        _ => None,
    };
    let title = format!(
        "Map ({})  {:.1} m view{}",
        robot.map(|(_, f)| f).unwrap_or("no pose"),
        view.map_zoom * 2.0,
        if view.map_follow { ", following robot" } else { "" }
    );
    let canvas = Canvas::default()
        .block(block(title))
        .marker(Marker::Braille)
        .x_bounds(xb)
        .y_bounds(yb)
        .paint(move |ctx| {
            ctx.draw(&Points { coords: &free, color: GRID });
            ctx.draw(&Points { coords: &occupied, color: Color::White });
            ctx.layer();
            for w in crumbs.windows(2) {
                ctx.draw(&CLine { x1: w[0].0, y1: w[0].1, x2: w[1].0, y2: w[1].1, color: Color::Yellow });
            }
            if let Some(first) = crumbs.first() {
                ctx.print(first.0, first.1, Span::styled("start", Style::new().fg(Color::Yellow)));
            }
            ctx.draw(&Points { coords: &scan_world, color: Color::Green });
            if let Some((p, _)) = robot {
                draw_robot(ctx, p);
            }
            if let Some((x, y)) = target {
                ctx.draw(&Circle { x, y, radius: 0.2, color: Color::Magenta });
            }
        });
    f.render_widget(canvas, canvas_area);

    let mut lines = Vec::new();
    match &st.map {
        Some(m) => {
            lines.push(kv("frame", m.frame.clone()));
            lines.push(kv("size", format!("{} x {} cells", m.width, m.height)));
            lines.push(kv("res", format!("{:.3} m/cell", m.resolution)));
            lines.push(kv("extent", format!("{:.1} x {:.1} m", m.width as f64 * m.resolution, m.height as f64 * m.resolution)));
            lines.push(kv("occupied", m.occupied.len().to_string()));
            lines.push(kv("unknown", m.unknown_cells.to_string()));
            lines.push(kv("updated", format!("{} ago", age_text(Some(m.at.elapsed())))));
        }
        None => {
            lines.push(Line::styled("no /map yet", Style::new().fg(Color::Yellow)));
            lines.push(Line::styled("slam_toolbox publishes every", Style::new().fg(DIM)));
            lines.push(Line::styled("few seconds once it has TF", Style::new().fg(DIM)));
        }
    }
    lines.push(Line::raw(""));
    match robot {
        Some((p, frame)) => {
            lines.push(kv("robot", format!("{frame} x {:+.2} y {:+.2}", p.x, p.y)));
            lines.push(kv("heading", format!("{:+.1} deg", p.yaw.to_degrees())));
        }
        None => lines.push(kv_styled("robot", "no odom/TF", Style::new().fg(Color::Red))),
    }
    lines.push(kv("crumbs", format!("{} in maps/breadcrumbs.json", st.breadcrumbs.len())));
    lines.push(Line::raw(""));
    for (parent, child) in [("map", "odom"), ("odom", "base_link"), ("base_link", "laser")] {
        let ok = st.tf.contains_key(&(parent.to_string(), child.to_string()));
        lines.push(kv_styled(
            if ok { "tf ok" } else { "tf MISSING" },
            format!("{parent} -> {child}"),
            Style::new().fg(if ok { Color::Green } else { Color::Red }),
        ));
    }
    lines.push(Line::raw(""));
    lines.push(Line::styled("white walls, green live scan,", Style::new().fg(DIM)));
    lines.push(Line::styled("yellow breadcrumbs, cyan robot", Style::new().fg(DIM)));
    f.render_widget(Paragraph::new(lines).block(block("Map info")), side);
}

fn draw_robot(ctx: &mut ratatui::widgets::canvas::Context, p: Pose2) {
    let corners = [(0.25, 0.0), (-0.15, 0.12), (-0.15, -0.12)];
    let pts: Vec<(f64, f64)> = corners.iter().map(|&(x, y)| p.apply(x, y)).collect();
    for i in 0..3 {
        let (a, b) = (pts[i], pts[(i + 1) % 3]);
        ctx.draw(&CLine { x1: a.0, y1: a.1, x2: b.0, y2: b.1, color: Color::Cyan });
    }
}

// ---------------------------------------------------------------- topics

fn draw_topics(f: &mut Frame, area: Rect, st: &State, view: &mut View) {
    let now = Instant::now();
    let mut names: Vec<String> = EXPECTED.iter().map(|(t, _, _)| t.to_string()).collect();
    for g in &st.graph {
        if !names.contains(&g.name) {
            names.push(g.name.clone());
        }
    }
    for t in st.topics.keys() {
        if !names.contains(t) {
            names.push(t.clone());
        }
    }
    let rows: Vec<Row> = names.iter().map(|name| {
        let expected = EXPECTED.iter().find(|(t, _, _)| t == name);
        let graph = st.graph.iter().find(|g| &g.name == name);
        let (status, style, hz, age) = topic_health(st, name, expected.and_then(|e| e.2), now);
        let count = st.topics.get(name).map(|s| s.count).unwrap_or(0);
        let note = st.topics.get(name).and_then(|s| s.error.clone()).or_else(|| expected.map(|e| e.1.to_string())).unwrap_or_default();
        let decoded = st.topics.contains_key(name) || expected.is_some();
        Row::new(vec![
            Cell::from(Span::styled(name.clone(), if decoded { Style::new() } else { Style::new().fg(Color::Gray) })),
            Cell::from(graph.map(|g| g.type_name.clone()).unwrap_or_else(|| "-".into())),
            Cell::from(graph.map(|g| g.publishers.to_string()).unwrap_or_else(|| "0".into())),
            Cell::from(graph.map(|g| g.subscribers.to_string()).unwrap_or_else(|| "0".into())),
            Cell::from(hz_text(hz)),
            Cell::from(count.to_string()),
            Cell::from(age_text(age)),
            Cell::from(Span::styled(status, style)),
            Cell::from(Span::styled(note, Style::new().fg(DIM))),
        ])
    }).collect();
    let height = area.height.saturating_sub(3) as usize;
    let max_scroll = rows.len().saturating_sub(height);
    view.scroll[3] = view.scroll[3].min(max_scroll);
    // Topics scroll from the top (unlike logs).
    let rows: Vec<Row> = rows.into_iter().skip(view.scroll[3]).collect();
    let table = Table::new(rows, [
        Constraint::Length(24),
        Constraint::Length(32),
        Constraint::Length(4),
        Constraint::Length(4),
        Constraint::Length(6),
        Constraint::Length(8),
        Constraint::Length(6),
        Constraint::Length(10),
        Constraint::Min(10),
    ])
    .header(Row::new(vec!["topic", "type (from discovery)", "pub", "sub", "Hz", "count", "age", "status", "note"]).style(Style::new().fg(DIM)))
    .block(block(format!("Topics: {} on the DDS graph, {} decoded here  (sub count includes this TUI)", st.graph.len(), st.topics.len())));
    f.render_widget(table, area);
}

// ---------------------------------------------------------------- tf

fn draw_tf(f: &mut Frame, area: Rect, st: &State, _view: &mut View) {
    let now = Instant::now();
    let [chain_area, table_area] = Layout::vertical([Constraint::Length(4), Constraint::Min(3)]).areas(area);
    let mut spans = Vec::new();
    for (i, (parent, child)) in [("map", "odom"), ("odom", "base_link"), ("base_link", "laser")].iter().enumerate() {
        let entry = st.tf.get(&(parent.to_string(), child.to_string()));
        let (mark, color) = match entry {
            Some(e) if e.is_static || e.stat.age(now).is_some_and(|a| a.as_secs_f64() < 1.0) => ("ok", Color::Green),
            Some(_) => ("STALE", Color::Yellow),
            None => ("MISSING", Color::Red),
        };
        if i == 0 {
            spans.push(Span::raw(parent.to_string()));
        }
        spans.push(Span::styled(format!(" -[{mark}]-> "), Style::new().fg(color).add_modifier(Modifier::BOLD)));
        spans.push(Span::raw(child.to_string()));
    }
    let lines = vec![
        Line::from(spans),
        Line::styled("map->odom comes from slam_toolbox, odom->base_link from encoder_odometry, base_link->laser is static.", Style::new().fg(DIM)),
    ];
    f.render_widget(Paragraph::new(lines).block(block("Expected chain")), chain_area);

    let rows: Vec<Row> = st.tf.iter().map(|((parent, child), e)| {
        let age = e.stat.age(now);
        let stale = !e.is_static && age.is_some_and(|a| a.as_secs_f64() > 1.0);
        Row::new(vec![
            Cell::from(format!("{parent} -> {child}")),
            Cell::from(format!("{:+.3}", e.pose.x)),
            Cell::from(format!("{:+.3}", e.pose.y)),
            Cell::from(format!("{:+.3}", e.z)),
            Cell::from(format!("{:+.1}", e.pose.yaw.to_degrees())),
            Cell::from(if e.is_static { "static".to_string() } else { hz_text(e.stat.hz(now)) }),
            Cell::from(Span::styled(age_text(age), Style::new().fg(if stale { Color::Red } else { Color::Reset }))),
        ])
    }).collect();
    let table = Table::new(rows, [
        Constraint::Length(30),
        Constraint::Length(9),
        Constraint::Length(9),
        Constraint::Length(9),
        Constraint::Length(8),
        Constraint::Length(8),
        Constraint::Min(6),
    ])
    .header(Row::new(vec!["parent -> child", "x m", "y m", "z m", "yaw deg", "Hz", "age"]).style(Style::new().fg(DIM)))
    .block(block(format!("Transforms ({})", st.tf.len())));
    f.render_widget(table, table_area);
}

// ---------------------------------------------------------------- logs

fn log_line(l: &LogLine, show_source: bool) -> Line<'static> {
    let (style, name) = level_style(l.level);
    let mut spans = vec![Span::styled(format!("{} ", l.time), Style::new().fg(DIM))];
    if show_source {
        spans.push(Span::styled(format!("{name} "), style));
        spans.push(Span::styled(format!("{}: ", l.source), Style::new().fg(Color::Cyan)));
    }
    spans.push(Span::styled(l.text.clone(), if l.level >= WARN { style } else { Style::new() }));
    Line::from(spans)
}

#[allow(clippy::too_many_arguments)]
fn draw_log(
    f: &mut Frame,
    area: Rect,
    ring: &Ring<LogLine>,
    view: &mut View,
    tab: usize,
    title: &str,
    keep: impl Fn(&LogLine) -> bool,
    show_source: bool,
) {
    debug_assert!(LOG_TABS.contains(&tab));
    // Hold position while scrolled back: shift by however many lines arrived.
    if view.scroll[tab] > 0 {
        view.scroll[tab] += ring.total.saturating_sub(view.last_total[tab]) as usize;
    }
    view.last_total[tab] = ring.total;
    let items: Vec<&LogLine> = ring.items.iter().filter(|l| keep(l)).collect();
    let height = area.height.saturating_sub(2) as usize;
    let max_scroll = items.len().saturating_sub(height);
    view.scroll[tab] = view.scroll[tab].min(max_scroll);
    let end = items.len() - view.scroll[tab];
    let start = end.saturating_sub(height);
    let lines: Vec<Line> = items[start..end].iter().map(|l| log_line(l, show_source)).collect();
    let position = if view.scroll[tab] > 0 { format!("  [scrolled back {}, End to follow]", view.scroll[tab]) } else { String::new() };
    f.render_widget(Paragraph::new(lines).block(block(format!("{title}  {} lines{position}", items.len()))), area);
}

#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    #[test]
    fn every_tab_renders_with_and_without_data() {
        for (w, h) in [(160, 48), (80, 24), (40, 12)] {
            let mut terminal = Terminal::new(TestBackend::new(w, h)).unwrap();
            let mut empty = State::new(42, 0.0);
            let mut full = State::new(42, 0.0);
            crate::demo::step(&mut full, 3.0);
            crate::demo::step(&mut full, 25.0);
            empty.rosout.push(LogLine { time: "00:00:00.000".into(), level: ERROR, source: "x".into(), text: "boom".into() });
            for st in [&mut empty, &mut full] {
                let mut view = View::default();
                for tab in 0..TABS.len() {
                    view.tab = tab;
                    terminal.draw(|f| draw(f, st, &mut view)).unwrap();
                    view.on_key(Key::PageUp, st);
                    view.on_key(Key::Char('-'), st);
                    view.on_key(Key::Left, st);
                    terminal.draw(|f| draw(f, st, &mut view)).unwrap();
                }
            }
        }
    }
}
