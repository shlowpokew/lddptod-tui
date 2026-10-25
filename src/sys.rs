//! Host-side checks that work even when DDS shows nothing: Pi health,
//! device nodes, the mapping container, its logs, and the breadcrumb file.

use std::io::{BufRead, BufReader};
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::thread;
use std::time::{Duration, Instant, SystemTime};

use crate::state::{clock, LogLine, Shared, ERROR, INFO, WARN};

const DEVICES: [&str; 5] = ["/dev/lidar", "/dev/uno", "/dev/i2c-1", "/dev/gpiochip4", "/dev/snd"];

fn read(path: &str) -> Option<String> {
    std::fs::read_to_string(path).ok()
}

fn run(cmd: &[String]) -> Option<String> {
    let (program, args) = cmd.split_first()?;
    let out = Command::new(program).args(args).stdin(Stdio::null()).stderr(Stdio::null()).output().ok()?;
    out.status.success().then(|| String::from_utf8_lossy(&out.stdout).trim().to_string())
}

fn words(parts: &[&str]) -> Vec<String> {
    parts.iter().map(|s| s.to_string()).collect()
}

/// `docker`, or `sudo -n docker` when the user isn't in the docker group.
pub fn docker_prefix() -> Option<Vec<String>> {
    for prefix in [words(&["docker"]), words(&["sudo", "-n", "docker"])] {
        let mut cmd = prefix.clone();
        cmd.extend(words(&["ps", "-q"]));
        if run(&cmd).is_some() {
            return Some(prefix);
        }
    }
    None
}

fn cpu_times() -> Option<(u64, u64)> {
    let stat = read("/proc/stat")?;
    let fields: Vec<u64> = stat.lines().next()?.split_whitespace().skip(1).filter_map(|v| v.parse().ok()).collect();
    let idle = fields.get(3)? + fields.get(4).unwrap_or(&0);
    Some((fields.iter().sum(), idle))
}

fn throttled_text(hex: &str) -> String {
    let value = u32::from_str_radix(hex.trim_start_matches("0x"), 16).unwrap_or(0);
    if value == 0 {
        return "ok".into();
    }
    let mut parts = Vec::new();
    for (bit, name) in [(0, "UNDERVOLT NOW"), (1, "freq capped"), (2, "THROTTLED NOW"), (3, "temp limit"), (16, "undervolt seen"), (18, "throttled seen")] {
        if value & (1 << bit) != 0 {
            parts.push(name);
        }
    }
    format!("{hex} {}", parts.join(", "))
}

pub fn spawn(shared: Shared, container: String, repo: PathBuf, docker_logs: bool) {
    let stats_shared = shared.clone();
    let stats_container = container.clone();
    let stats_repo = repo.clone();
    thread::Builder::new()
        .name("sys".into())
        .spawn(move || stats_loop(stats_shared, stats_container, stats_repo))
        .expect("spawn sys thread");
    if docker_logs {
        thread::Builder::new()
            .name("docker-logs".into())
            .spawn(move || logs_loop(shared, container))
            .expect("spawn docker logs thread");
    }
}

fn stats_loop(shared: Shared, container: String, repo: PathBuf) {
    let hostname = read("/etc/hostname").map(|h| h.trim().to_string()).unwrap_or_default();
    let mut docker = docker_prefix();
    let mut last_cpu = cpu_times();
    let mut tick: u64 = 0;
    let crumbs = repo.join("maps").join("breadcrumbs.json");
    let mut crumbs_mtime: Option<SystemTime> = None;
    loop {
        let mut info = shared.lock().unwrap().sys.clone();
        info.hostname = hostname.clone();
        info.cpu_temp_c = read("/sys/class/thermal/thermal_zone0/temp")
            .and_then(|t| t.trim().parse::<f64>().ok())
            .map(|m| m / 1000.0);
        let cpu = cpu_times();
        if let (Some((t0, i0)), Some((t1, i1))) = (last_cpu, cpu) {
            let (dt, di) = (t1.saturating_sub(t0), i1.saturating_sub(i0));
            if dt > 0 {
                info.cpu_percent = Some(100.0 * (dt - di.min(dt)) as f64 / dt as f64);
            }
        }
        last_cpu = cpu;
        info.load = read("/proc/loadavg")
            .map(|l| l.split_whitespace().take(3).collect::<Vec<_>>().join(" "))
            .unwrap_or_default();
        if let Some(mem) = read("/proc/meminfo") {
            let field = |name: &str| {
                mem.lines()
                    .find(|l| l.starts_with(name))
                    .and_then(|l| l.split_whitespace().nth(1))
                    .and_then(|v| v.parse::<u64>().ok())
            };
            if let (Some(total), Some(avail)) = (field("MemTotal:"), field("MemAvailable:")) {
                info.mem_used_mb = Some(((total - avail.min(total)) / 1024, total / 1024));
            }
        }
        info.uptime = read("/proc/uptime")
            .and_then(|u| u.split_whitespace().next()?.parse::<f64>().ok())
            .map(Duration::from_secs_f64);
        info.devices = DEVICES
            .iter()
            .filter_map(|d| {
                let path = std::path::Path::new(d);
                if !path.exists() && !matches!(*d, "/dev/lidar" | "/dev/uno") {
                    return None;
                }
                let target = std::fs::read_link(path)
                    .ok()
                    .map(|t| t.display().to_string())
                    .or_else(|| path.exists().then(|| "present".to_string()));
                Some((d.to_string(), target))
            })
            .collect();

        // Slower checks every 5 s.
        if tick % 5 == 0 {
            info.throttled = run(&words(&["vcgencmd", "get_throttled"]))
                .and_then(|o| o.split('=').nth(1).map(throttled_text));
            info.disk = run(&words(&["df", "-hP", "/"])).and_then(|o| {
                let line = o.lines().nth(1)?.to_string();
                let f: Vec<&str> = line.split_whitespace().collect();
                Some(format!("{} free of {} ({} used)", f.get(3)?, f.get(1)?, f.get(4)?))
            });
            info.git = run(&[
                "git".into(), "-C".into(), repo.display().to_string(),
                "log".into(), "-1".into(), "--format=%h %cr: %s".into(),
            ]);
            if docker.is_none() {
                docker = docker_prefix();
            }
            match &docker {
                Some(prefix) => {
                    let mut cmd = prefix.clone();
                    cmd.extend(words(&["ps", "-a", "--filter"]));
                    cmd.push(format!("name=^{container}$"));
                    cmd.extend(words(&["--format", "{{.Status}}"]));
                    info.container = run(&cmd).map(|s| if s.is_empty() { "no such container".into() } else { s });
                    info.docker_note = (prefix[0] == "sudo").then(|| "via sudo -n".into());
                }
                None => {
                    info.container = None;
                    info.docker_note = Some("no access (add user to docker group or run with sudo)".into());
                }
            }
        }

        let mtime = std::fs::metadata(&crumbs).and_then(|m| m.modified()).ok();
        let new_crumbs = if mtime != crumbs_mtime {
            crumbs_mtime = mtime;
            Some(load_breadcrumbs(&crumbs))
        } else {
            None
        };

        {
            let mut st = shared.lock().unwrap();
            st.sys = info;
            if let Some(c) = new_crumbs {
                st.breadcrumbs = c;
            }
        }
        tick += 1;
        thread::sleep(Duration::from_secs(1));
    }
}

fn load_breadcrumbs(path: &std::path::Path) -> Vec<(f64, f64)> {
    let Ok(text) = std::fs::read_to_string(path) else { return Vec::new() };
    let Ok(serde_json::Value::Array(items)) = serde_json::from_str::<serde_json::Value>(&text) else {
        return Vec::new();
    };
    items
        .iter()
        .filter_map(|c| Some((c.get("x")?.as_f64()?, c.get("y")?.as_f64()?)))
        .collect()
}

/// Strips terminal colour codes from docker output.
fn strip_ansi(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\u{1b}' {
            if chars.peek() == Some(&'[') {
                chars.next();
                for c in chars.by_ref() {
                    if c.is_ascii_alphabetic() {
                        break;
                    }
                }
            }
        } else {
            out.push(c);
        }
    }
    out
}

pub fn classify(line: &str) -> u8 {
    if line.contains("[FATAL]") || line.contains("[ERROR]") || line.contains("Traceback") || line.contains("Error:") || line.contains("exited with") {
        ERROR
    } else if line.contains("[WARN]") || line.contains("WARNING") {
        WARN
    } else {
        INFO
    }
}

fn push_docker(shared: &Shared, level: Option<u8>, line: &str) {
    let text = strip_ansi(line);
    let level = level.unwrap_or_else(|| classify(&text));
    shared.lock().unwrap().docker.push(LogLine { time: clock(), level, source: String::new(), text });
}

fn logs_loop(shared: Shared, container: String) {
    loop {
        let Some(prefix) = docker_prefix() else {
            push_docker(&shared, Some(WARN), "[tui] can't run docker (not in docker group and no passwordless sudo); retrying in 10 s");
            thread::sleep(Duration::from_secs(10));
            continue;
        };
        let mut cmd = prefix.clone();
        cmd.extend(words(&["logs", "-f", "--tail", "300", &container]));
        let started = Instant::now();
        let child = Command::new(&cmd[0])
            .args(&cmd[1..])
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn();
        match child {
            Ok(mut child) => {
                push_docker(&shared, Some(INFO), &format!("[tui] following {}", cmd.join(" ")));
                // ROS logs go to the container's stderr, so read both.
                let stderr = child.stderr.take();
                let err_shared = shared.clone();
                let err_thread = thread::spawn(move || {
                    if let Some(stderr) = stderr {
                        for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                            push_docker(&err_shared, None, &line);
                        }
                    }
                });
                if let Some(stdout) = child.stdout.take() {
                    for line in BufReader::new(stdout).lines().map_while(Result::ok) {
                        push_docker(&shared, None, &line);
                    }
                }
                let _ = err_thread.join();
                let status = child.wait().map(|s| s.to_string()).unwrap_or_default();
                push_docker(&shared, Some(WARN), &format!("[tui] docker logs ended ({status}); retrying in 3 s"));
            }
            Err(e) => push_docker(&shared, Some(ERROR), &format!("[tui] {}: {e}", cmd.join(" "))),
        }
        if started.elapsed() < Duration::from_secs(3) {
            thread::sleep(Duration::from_secs(3));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strips_colour() {
        assert_eq!(strip_ansi("\u{1b}[31m[ERROR]\u{1b}[0m boom"), "[ERROR] boom");
    }

    #[test]
    fn throttle_bits() {
        assert_eq!(throttled_text("0x0"), "ok");
        assert!(throttled_text("0x50005").contains("UNDERVOLT NOW"));
    }
}
